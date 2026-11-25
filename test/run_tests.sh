#!/bin/sh
# Runs the host tests. Needs: g++, python3 + numpy, and checkouts of
# tone-3000/embedded and sdatkinson/NeuralAmpModelerCore (for the test captures).
#   EMB=../../ext/nam-embedded CORE=../../ext/NeuralAmpModelerCore ./run_tests.sh
set -e
EMB=${EMB:-../../ext/nam-embedded}; CORE=${CORE:-../../ext/NeuralAmpModelerCore}
W=$(mktemp -d); mkdir -p $W/namdir $W/namb_only
make -s
cp $CORE/example_models/A2.nam "$W/namdir/Example A2 capture.nam"
cp $EMB/weights/a2lite.namb $W/namdir/z_lite.namb; cp $EMB/weights/a2full.namb $W/namdir/full_model.namb
cp $CORE/example_models/wavenet_a1_standard.nam $W/namdir/old_a1.nam; cp $EMB/weights/a2lite.namb $W/namb_only/
python3 -c "
import json,numpy as np,sys
d=json.load(open('$CORE/example_models/A2.nam'))
np.array(d['config']['submodels'][0]['model']['weights'],dtype=np.float64).astype(np.float32).tofile('$W/ref_w.f32')
(np.fromfile('$EMB/test/golden_input.raw',dtype=np.float32)*5).astype(np.float32).tofile('$W/in.f32')"
echo "== parser"; ./host_test parse "$W/namdir/Example A2 capture.nam" $W/ref_w.f32
python3 -c "
import numpy as np; (np.fromfile('$W/in.f32',dtype=np.float32)*0.5).astype(np.float32).tofile('$W/half.f32')"
# esr <file> <reference> <scale>: ESR of file/scale against reference
esr() { python3 -c "
import numpy as np; a=np.fromfile('$1',dtype=np.float32)/$3; b=np.fromfile('$2',dtype=np.float32); n=min(len(a),len(b)); e=a[:n]-b[:n]
v=10*np.log10((e**2).sum()/(b[:n]**2).sum()+1e-40); print('    %-34s ESR %7.1f dB  max err %.1e  %s'%('$4',v,abs(e).max(),'ok' if v<-100 else 'FAIL'))"; }
G=$EMB/test/golden_a2lite_output.raw
R() { ./host_test render $W/namb_only 0 "$@" >/dev/null; }
echo "== mono spec vs NeuralAmpModelerCore golden"
for b in 24 128; do R 1 0 $W/in.f32 - $W/m.f32 - $b; esr $W/m.f32 $G 5 "block $b"; done
echo "== stereo spec"
R 2 0 $W/in.f32 $W/half.f32 $W/l.f32 $W/r.f32; R 1 0 $W/half.f32 - $W/mh.f32 -
esr $W/l.f32 $G 5 "dual: L vs golden"; esr $W/r.f32 $W/mh.f32 1 "dual: R (half level) vs mono run"
R 2 1 $W/in.f32 $W/in.f32 $W/l.f32 $W/r.f32
esr $W/l.f32 $G 5 "summed L=R: L vs golden"; esr $W/r.f32 $G 5 "summed L=R: R vs golden"
R 2 0 $W/in.f32 - $W/l.f32 $W/r.f32
esr $W/l.f32 $G 5 "mono in > stereo: L vs golden"; esr $W/r.f32 $G 5 "mono in > stereo: R vs golden"
echo "== stereo spec @96k (dual R must equal a mono 96k run)"
python3 -c "
import numpy as np, scipy.signal as s
for f in ['in','half']: s.resample_poly(np.fromfile('$W/'+f+'.f32',dtype=np.float32),2,1).astype(np.float32).tofile('$W/'+f+'96.f32')"
NT_SR=96000 ./host_test render $W/namb_only 0 2 0 $W/in96.f32 $W/half96.f32 $W/l.f32 $W/r.f32 >/dev/null
NT_SR=96000 ./host_test render $W/namb_only 0 1 0 $W/half96.f32 - $W/mh.f32 - >/dev/null
NT_SR=96000 ./host_test render $W/namb_only 0 1 0 $W/in96.f32 - $W/ml.f32 - >/dev/null
esr $W/l.f32 $W/ml.f32 1 "dual 96k: L vs mono 96k"; esr $W/r.f32 $W/mh.f32 1 "dual 96k: R vs mono 96k"
echo "== behaviour @48k"; ./host_test misc $W/namdir
echo "== behaviour @96k"; NT_SR=96000 ./host_test misc $W/namdir
rm -rf $W
