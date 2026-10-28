"""Standard-library-only static publication check; this does not run MATLAB."""
from pathlib import Path
import hashlib,json,zipfile
root=Path(__file__).resolve().parents[1]
m=json.loads((root/'reference_data/source_manifest.json').read_text(encoding='utf-8'))
for name,expected in m['kernelSha256'].items():
    assert hashlib.sha256((root/'src'/name).read_bytes()).hexdigest()==expected,name
for name,expected in m['mssFileSha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
assert hashlib.sha256((root/'models/auv_v3_closed_loop.slx').read_bytes()).hexdigest()==m['modelSha256']
files=list((root/'reference_data/v3').glob('*.mat'));assert len(files)==24
for p in root.rglob('*'):
    if not p.is_file() or '.git' in p.parts or p.parts[len(root.parts)]=='results':continue
    assert p.stat().st_size<50*1024*1024,p
    if p.suffix in {'.m','.md','.json','.py','.cff','.csv'}:
        text=p.read_text(encoding='utf-8-sig')
        assert (chr(67)+':'+chr(92)+'Users'+chr(92)) not in text and '/'.join(['C:','Users','']) not in text,p
for name, expected in json.loads((root/'reference_data/numeric_reference_sha256.json').read_text()).items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==expected,name
assert (root/'references/code/MSS/LICENSE').is_file()
with zipfile.ZipFile(root/'models/auv_v3_closed_loop.slx') as z:
    assert all(b'31369' not in z.read(n) for n in z.namelist() if n.endswith(('.xml','.rels')))
print('PASS: kernel/MSS/model hashes, 24 numeric references, size and portable-source checks.')
