#!/usr/bin/env python3
import re, sys, statistics as st
N = r'([-+0-9.eE]+)'
V = r'\{\s*x: ' + N + r'\s*y: ' + N + r'\s*z: ' + N
agg = {}
for b in re.split(r'(?m)^contact \{', open(sys.argv[1]).read())[1:]:
    m = re.search(r'collision2: "([^"]+)"', b)
    if not m:
        continue
    k = '::'.join(m.group(1).split('::')[:2])
    a = agg.setdefault(k, dict(steps=0, p=[], n=[], d=[]))
    a['steps'] += 1
    a['p'] += [tuple(map(float, x)) for x in re.findall('position ' + V, b)]
    a['n'] += [tuple(map(float, x)) for x in re.findall('normal ' + V, b)]
    a['d'] += [float(x) for x in re.findall('depth: ' + N, b)]
for k, a in agg.items():
    mean = lambda L, i: sum(v[i] for v in L) / max(len(L), 1)
    d = sorted(a['d'])
    print(k, 'steps', a['steps'])
    print('  pos (%.4f %.4f %.4f)  normal (%.3f %.3f %.3f)' % (
        *(mean(a['p'], i) for i in range(3)), *(mean(a['n'], i) for i in range(3))))
    print('  depth median %.4f  p95 %.4f  max %.4f' % (st.median(d), d[int(0.95 * len(d))], d[-1]))
