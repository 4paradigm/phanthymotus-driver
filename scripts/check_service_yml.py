#!/usr/bin/env python3
"""Check every driver's deploy/service.yml for the DDS isolation contract.

Run this on any PR that adds or edits a service.yml. Exits non-zero on a violation,
so it also works as a CI step.

    ./scripts/check_service_yml.py            # all drivers
    ./scripts/check_service_yml.py unitree/g1 # just one

What it enforces, and why each item is worth a check rather than a README line:

  * the loopback profile is mounted, and (unless the driver picks profiles in code)
    selected via FASTRTPS_DEFAULT_PROFILES_FILE. Miss these and the container ends up
    on a different transport from everything else on the machine, so it cannot reach
    Agent Core at all. The failure does not look like "not isolated" — the device
    registers over HTTP and shows up in the dashboard while none of its topics ever
    carry data, which sends you looking at the driver instead of at compose.

  * FASTDDS_BUILTIN_TRANSPORTS is absent. It contradicts the profile's
    useBuiltinTransports=false, and the XML wins, so the variable does nothing except
    tell the next reader something untrue.

  * domain 42, spelled either ROS_DOMAIN_ID or <PREFIX>_ROS_DOMAIN_ID for a driver
    that holds two contexts. There is nothing to allocate here; a different number is
    a mistake, not a choice.

  * network_mode: host. Isolation works by confining DDS to loopback, and containers
    share a loopback only under host networking.

KNOWN_GAPS below is the honest part: a driver whose RMW is CycloneDDS is not covered by
a FastDDS profile at all, and saying so out loud beats letting it pass a check named
"isolation". Those are reported and counted, but do not fail the run.
"""

import os
import sys
import glob

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required: pip3 install pyyaml")

PROFILE = '/opt/phanthy-motus/dds-local.xml'
MOUNT = f'{PROFILE}:{PROFILE}:ro'
DOMAIN = '42'

# Drivers that must NOT set FASTRTPS_DEFAULT_PROFILES_FILE, with the reason. A
# process-wide default applies to every participant in the process, so a driver holding
# a body context and an Agent Core context in one process has to select per context —
# it sets the variable around each rclpy.init() instead. The mount is still required.
IN_CODE_PROFILE = {
    'x-humanoid/tianyi2.0': 'DualDomainROS2 in main.py sets the variable per rclpy.init() '
                            '(body on domain 0 via the vendor profile, core on 42)',
}

# Drivers a FastDDS profile cannot isolate, with what they would need instead. Reported,
# not fatal — the point is that the gap stays visible rather than passing as compliant.
KNOWN_GAPS = {
    'engineai/t800': 'RMW is rmw_cyclonedds_cpp (Dockerfile T800_PREFERRED_RMW), so '
                     'FASTRTPS_DEFAULT_PROFILES_FILE is inert; its CYCLONEDDS_URI binds '
                     'NETWORK_INTERFACE for both contexts, leaving the domain-42 one on '
                     'the LAN. Needs a CycloneDDS-level restriction for the core context.',
}


def env_map(env):
    """Accept both list-of-KEY=VALUE and mapping forms; compose allows either."""
    if isinstance(env, dict):
        return {str(k): '' if v is None else str(v) for k, v in env.items()}
    out = {}
    for item in env or []:
        k, _, v = str(item).partition('=')
        out[k.strip()] = v.strip()
    return out


def check(path, driver):
    """Return (violations, notes) for one service.yml."""
    bad, notes = [], []
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    if len(doc) != 1:
        return [f'expected exactly one service, found {len(doc)}'], notes
    svc = next(iter(doc.values())) or {}

    volumes = [str(v) for v in (svc.get('volumes') or [])]
    env = env_map(svc.get('environment'))

    gap = KNOWN_GAPS.get(driver)

    if MOUNT not in volumes:
        # A read-write mount also works, but ro is the intent and a typo'd path is the
        # failure mode we are actually guarding against: Docker silently creates a
        # *directory* of that name, FastDDS falls back to every interface, nothing logs.
        loose = [v for v in volumes if PROFILE in v]
        if loose:
            bad.append(f'profile mounted as {loose[0]!r}, expected {MOUNT!r}')
        elif not gap:
            bad.append(f'missing volume {MOUNT!r} — without it the container is not isolated, '
                       'and a mistyped path fails silently')

    reason = IN_CODE_PROFILE.get(driver)
    have_var = env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if reason:
        if have_var:
            bad.append(f'must NOT set FASTRTPS_DEFAULT_PROFILES_FILE: {reason}')
        else:
            notes.append(f'profile selected in code — {reason}')
    elif not gap:
        if not have_var:
            bad.append('missing FASTRTPS_DEFAULT_PROFILES_FILE — the mount alone selects nothing')
        elif have_var != PROFILE:
            bad.append(f'FASTRTPS_DEFAULT_PROFILES_FILE={have_var}, expected {PROFILE}')

    if 'FASTDDS_BUILTIN_TRANSPORTS' in env:
        bad.append('remove FASTDDS_BUILTIN_TRANSPORTS — it conflicts with the profile\'s '
                   'useBuiltinTransports=false and the XML wins anyway')

    # Either the plain name, or a prefixed one for a driver with a second context.
    domains = {k: v for k, v in env.items() if k.endswith('ROS_DOMAIN_ID')}
    core = [v for k, v in domains.items()
            if k == 'ROS_DOMAIN_ID' or k.startswith(('AGENT_CORE', 'CORE'))]
    if core and DOMAIN not in core:
        bad.append(f'Agent Core domain is {core}, expected {DOMAIN} — the same on every robot')
    elif not domains and not reason and not gap:
        notes.append('no ROS_DOMAIN_ID in the fragment; confirm it is set in the image')

    # network_mode: host is load-bearing here, not boilerplate — isolation works by
    # confining DDS to loopback, and containers only share a loopback under host
    # networking. All 14 drivers set it. ipc/pid are deliberately *not* checked: they
    # vary across drivers, the profile disables shared memory anyway, and failing five
    # existing drivers over them on day one is how a checker earns being ignored.
    if svc.get('network_mode') != 'host':
        bad.append(f'network_mode is {svc.get("network_mode")!r}, expected \'host\' — '
                   'containers share a loopback only under host networking, and loopback '
                   'is what makes local peers reachable while the LAN is not')

    if gap:
        notes.append(f'KNOWN GAP — {gap}')
    return bad, notes


def main(argv):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if argv:
        paths = [os.path.join(root, a, 'deploy', 'service.yml') for a in argv]
        for p in paths:
            if not os.path.exists(p):
                sys.exit(f'no such service.yml: {p}')
    else:
        paths = sorted(glob.glob(os.path.join(root, '*', '*', 'deploy', 'service.yml')))

    failed = gaps = 0
    for path in paths:
        driver = os.path.relpath(os.path.dirname(os.path.dirname(path)), root)
        try:
            bad, notes = check(path, driver)
        except Exception as exc:                      # noqa: BLE001 — report, keep going
            print(f'FAIL {driver}\n       could not parse: {exc}')
            failed += 1
            continue
        if bad:
            failed += 1
            print(f'FAIL {driver}')
            for b in bad:
                print(f'       {b}')
        elif any(n.startswith('KNOWN GAP') for n in notes):
            gaps += 1
            print(f'GAP  {driver}')
        else:
            print(f'ok   {driver}')
        for n in notes:
            print(f'       note: {n}')

    print(f'\n{len(paths)} checked, {failed} failed, {gaps} known gap(s)')
    if failed:
        print('See README_dev.md § "DDS isolation" for what each item is protecting against.')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
