"""Keep the 2D enrollment launcher outside the immersive activity."""
from pathlib import Path
import xml.etree.ElementTree as E
import struct
import subprocess
base=Path(__file__).resolve().parents[1]/'app/src'
a='{http://schemas.android.com/apk/res/android}'
app=E.parse(base/'main/AndroidManifest.xml').getroot().find('application')
launcher=next(x for x in app.findall('activity') if x.get(a+'name').endswith('ConnectionActivity'))
assert {x.get(a+'name') for x in launcher.iter('category')}=={'android.intent.category.LAUNCHER', 'android.intent.category.DEFAULT', 'android.intent.category.BROWSABLE'}
assert any(x.get(a+'scheme')=='motus-teleop' and x.get(a+'host')=='connect' for x in launcher.iter('data'))
assert next(x for x in app.findall('activity') if x.get(a+'name')=='android.app.NativeActivity').get(a+'permission')=='android.permission.DUMP'
pico=E.parse(base/'pico/AndroidManifest.xml').getroot().find('application')
assert not pico.findall('meta-data')
native=pico.find('activity')
assert native.get(a+'name')=='android.app.NativeActivity'
assert any(x.get(a+'name')=='pvr.app.type' and x.get(a+'value')=='vr' for x in native.findall('meta-data'))
assert app.get(a+'icon') == '@mipmap/ic_launcher'
for density, size in [('mdpi',48),('hdpi',72),('xhdpi',96),('xxhdpi',144),('xxxhdpi',192)]:
    icon = base / f'main/res/mipmap-{density}/ic_launcher.png'
    content = icon.read_bytes()
    assert content[:8] == b'\x89PNG\r\n\x1a\n'
    assert struct.unpack('>II', content[16:24]) == (size, size)
    # An ignored asset may work locally but disappear from a clean PR build.
    assert subprocess.run(['git','check-ignore','--quiet',str(icon)], cwd=base).returncode == 1
print('Launcher manifest and distributable icon checks PASS')
