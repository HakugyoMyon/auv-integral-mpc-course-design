"""Verify the archived manufacturer workbook -> exact 16 V MIL lookup table.
Run with Python 3; standard library only. No writes or network access.
"""
from pathlib import Path
import csv
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile

root = Path(__file__).resolve().parent
record = json.loads((root / 'T200_data_provenance.json').read_text(encoding='utf-8'))
original = root / 'T200_Performance_2019.xlsx'
derived = root.parent / record['derived_csv']
assert hashlib.sha256(original.read_bytes()).hexdigest() == record['original_sha256']
assert hashlib.sha256(derived.read_bytes()).hexdigest() == record['derived_sha256']
ns = {'s': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
rows = []
with zipfile.ZipFile(original) as book:
    data = ET.fromstring(book.read(record['worksheet_part']))
    for row in data.findall('s:sheetData/s:row', ns):
        cells = {c.attrib['r'].rstrip('0123456789'): c.find('s:v', ns).text
                 for c in row.findall('s:c', ns)
                 if c.find('s:v', ns) is not None and c.attrib.get('t') != 's'}
        if all(k in cells for k in 'ABCDEF'):
            values = [float(cells[k]) for k in 'ABCDEF']
            rows.append(values + [values[-1] * 9.80665])
with derived.open(encoding='utf-8-sig', newline='') as stream:
    actual = list(csv.DictReader(stream))
assert len(actual) == len(rows) == record['rows']
error = max(abs(float(a[key]) - value)
            for a, values in zip(actual, rows)
            for key, value in zip(record['columns'], values))
assert error < 1e-10
print(f'PASS: manufacturer SHA-256, {len(rows)} rows, max numeric difference {error:g}')
