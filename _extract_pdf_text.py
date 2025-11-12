import sys, subprocess
try:
    import pypdf
except Exception:
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--quiet', 'pypdf'])
    import pypdf
from pypdf import PdfReader
reader = PdfReader('Documentation/controlbook.pdf')
text_parts = []
for i, page in enumerate(reader.pages):
    try:
        t = page.extract_text() or ''
    except Exception:
        t = ''
    text_parts.append(t)
text = "\n".join(text_parts)
open('_controlbook.txt', 'w', encoding='utf-8').write(text)
print('WROTE', len(text), 'chars to _controlbook.txt')
