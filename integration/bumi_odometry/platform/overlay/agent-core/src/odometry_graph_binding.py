"""Authoritative saved-canvas leases for opt-in odometry feedback consumers.

Deployed with the companion platform patch, not imported by ordinary cards.
No new network endpoint, auth exemption or cross-tool subscription discovery.
"""
import asyncio
import hashlib
import json
import time

_task=None
_last_issued=0
_failures={}


def binding_for(layout,card,*,now_ns=None,find_tool=None):
    issued=now_ns if now_ns is not None else time.time_ns()
    cards={c.get('id'):c for c in layout.get('cards',[])}
    edges=[c for c in layout.get('connections',[]) if c.get('toCardId')==card.get('id')]
    identities=[]; topics=[]; valid=len(edges)<=1
    for edge in edges:
        source=cards.get(edge.get('fromCardId'),{})
        ports=source.get('topicOut') or []
        try: index=int(edge.get('fromPortIdx',0)); destination=int(edge.get('toPortIdx',0))
        except (TypeError,ValueError): index=-1; destination=-1
        declaration=None
        if find_tool and source:
            current=find_tool(source.get('mcpId',''),source.get('toolName','')) or {}
            live=current.get('topic_out') or []
            if 0<=index<len(live): declaration=live[index]
            if (not declaration or declaration.get('format')!='state/odom'
                    or declaration.get('message_type','std_msgs/msg/String')!='std_msgs/msg/String'
                    or not declaration.get('topic')):
                valid=False
        saved=ports[index] if 0<=index<len(ports) else {}
        topic=edge.get('fromTopic') or saved.get('topic') or (declaration or {}).get('topic') or ''
        fmt=edge.get('format') or saved.get('format') or (declaration or {}).get('format')
        valid=valid and source.get('toolName')=='rgbd_imu_odometry' and destination==0 and index>=0
        valid=valid and fmt=='state/odom' and isinstance(topic,str) and topic.startswith('/')
        if declaration and declaration.get('topic') and topic!=declaration['topic']:
            valid=False  # stale saved port must never select another publisher
        identities.append([edge.get('id'),edge.get('fromCardId'),source.get('mcpId'),source.get('toolName'),
                           index,card.get('id'),destination,fmt,topic])
        topics.append(topic)
    revision=hashlib.sha256(json.dumps([card.get('id'),sorted(identities,key=str)],sort_keys=True).encode()).hexdigest()
    return dict(version=1,revision=revision,input_topic=topics[0] if len(topics)==1 else '',
                resolved=bool(valid),issued_ns=issued,expires_ns=issued+500_000_000)


def opted_in(mcp_id,tool):
    from tool_config import find_tool
    return (find_tool(mcp_id,tool).get('inputSchema') or {}).get('x-feedback-binding')==1


def saved_binding(mcp_id,tool,instance_id=''):
    import config
    from tool_config import find_tool
    layout=config.main.get('canvas_layout',{}) or {}
    matches=[c for c in layout.get('cards',[]) if c.get('mcpId')==mcp_id and c.get('toolName')==tool
             and (not instance_id or c.get('id')==instance_id)]
    if len(matches)!=1: raise ValueError('feedback requires exactly one saved consumer card')
    global _last_issued
    _last_issued=max(time.time_ns(),_last_issued+1)
    return binding_for(layout,matches[0],now_ns=_last_issued,find_tool=find_tool)


def payload(response):
    data=response.get('data',{})
    result=data.get('result',data)
    if 'content' in result:
        for block in result['content']:
            if block.get('type')=='text': return json.loads(block['text'])
    return result


async def refresh(force=False):
    import config
    from api.mcp_manage import mcp_call_tool,MCPCallRequest
    if not force and not (config.main.get('core',{}) or {}).get('project_running',False): return
    layout=config.main.get('canvas_layout',{}) or {}
    targets=[c for c in layout.get('cards',[]) if opted_in(c.get('mcpId',''),c.get('toolName',''))]
    async def deliver(card):
        key=(card['mcpId'],card['toolName'],card['id'])
        try:
            binding=saved_binding(*key)
            request=MCPCallRequest(tool=key[1],arguments={
                'action':'config','instance_id':key[2],'feedback_binding':binding})
            response=await mcp_call_tool(key[0],request,timeout_s=.2)
            # The authoritative MCP handler may replace the supplied lease.
            binding=request.arguments['feedback_binding']
            answer=payload(response)
            ports=answer.get('topic_in') or []
            confirmed_topic=ports[0].get('topic','') if len(ports)==1 else None
            if (answer.get('state')!='ready' or answer.get('feedback_binding')!=binding
                    or confirmed_topic!=binding['input_topic']):
                raise RuntimeError('consumer did not acknowledge saved connection')
            _failures.pop(key,None)
        except Exception as exc:
            if key not in _failures: print('[feedback-binding] '+str(key)+': '+str(exc))
            _failures[key]=str(exc)
            # A lost ACK can still mean the driver accepted the lease. Revoke
            # it before retrying; an unreachable driver expires its lease.
            try:
                await mcp_call_tool(key[0],MCPCallRequest(tool=key[1],arguments={
                    'action':'stop','instance_id':key[2]}),timeout_s=.2)
            except Exception:
                pass
    await asyncio.gather(*(deliver(c) for c in targets))


def start():
    global _task
    if _task is not None and not _task.done(): return
    async def watch():
        while True:
            try: await refresh()
            except Exception as exc: print('[feedback-binding] refresh: '+str(exc))
            await asyncio.sleep(.1)
    _task=asyncio.create_task(watch())
