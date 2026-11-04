"""Restricted SSH key entry point: accepts only 'fr3-backend <allowed IP>'."""
import os
from pathlib import Path
import shlex
import sys

def requested_robot(command, allowed):
    words=shlex.split(command)
    if len(words)!=2 or words[0]!='fr3-backend' or words[1] not in allowed:
        raise ValueError('Only an allowlisted FR3 backend session is permitted')
    return words[1]

if __name__=='__main__':
    allowed = sys.argv[1:]  # Fixed arguments in the administrator's forced command.
    try: address=requested_robot(os.environ.get('SSH_ORIGINAL_COMMAND',''), allowed)
    except ValueError as error:
        print(error,file=sys.stderr);sys.exit(1)
    base=Path(__file__).resolve().parent
    os.execv(sys.executable,[sys.executable,str(base/'remote_backend.py'),'server',address,
        '--allowed',','.join(allowed),'--native',str(base/'build/franka-state-reader')])
