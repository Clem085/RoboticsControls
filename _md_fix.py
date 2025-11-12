from pathlib import Path
p=Path('Documentation/HW7.md')
s=p.read_text(encoding='utf-8')
hdr='### How to run (verification)'
idxs=[]
i=0
while True:
    j=s.find(hdr,i)
    if j==-1: break
    idxs.append(j)
    i=j+1
if len(idxs)>=2:
    start=idxs[1]
    end=s.find('### Placeholders', start)
    if end==-1:
        end=len(s)
    new_block='''### How to run (verification)

- Dedicated script now included: _F_planar_vtol/python/hwF11_VTOLSim.py (uses _F_planar_vtol/python/ctrlFSF.py).

Run command:

`ash
python _F_planar_vtol/python/hwF11_VTOLSim.py
`

CMD Output
`ash
(.venv) C:\\Users\\consa\\Downloads\\Programming\\Robotics_Controls>C:/Users/consa/Downloads/Programming/Robotics_Controls/.venv/Scripts/python.exe c:/Users/consa/Downloads/Programming/Robotics_Controls/_F_planar_vtol/python/hwF11_VTOLSim.py

===== F.11 Full-State Feedback (VTOL) =====
Desired poles: [p1, p2, p3, p4, p5, p6]
Closed-loop poles: [ ... six values ... ]
K =
 [[ ... 2x6 gain matrix ... ]]
Kr =
 [[ ... 2x2 reference gain ... ]]
Controllability rank = 6
Press key to close
`

GUI Output
![alt text](image-3.png)
'''
    s = s[:start] + new_block + s[end:]
    p.write_text(s, encoding='utf-8')
    print('UPDATED')
else:
    print('HEADER NOT FOUND OR COUNT < 2')
