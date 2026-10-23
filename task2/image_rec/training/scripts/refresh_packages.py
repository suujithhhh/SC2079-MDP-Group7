"""Refresh portable runtime/report files and checksums without rerunning model evaluation."""
from pathlib import Path
import hashlib,json,shutil,zipfile
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 r=json.loads((ROOT/'reports/results.json').read_text());summaries=[]
 for size in [640,512]:
  folder=ROOT/'handoff'/f'task2_yolo26n_{size}_ncnn_model'
  shutil.copy2(ROOT/'scripts/inference.py',folder/'inference.py')
  shutil.copy2(ROOT/'reports/results.json',folder/'validation.json')
  shutil.copy2(ROOT/'reports/smoke_test.json',folder/'smoke_test_640.json')
  status='Recommended candidate for live Pi testing.' if size==r['recommended_imgsz'] else 'Comparison export only: not recommended. It produced an opposite-arrow error during validation.'
  (folder/'README.txt').write_text(f'''Task2 YOLO26n detection: bullseye / left / right
{status}

Fixed network input: {size}x{size}. Confirmation confidence: {r['confidence']}.
Classes0/1/2 map to legacy IDs marker/39/38. Use this detection runner;
the former31-class segmentation decoder is not interchangeable.

Run from this folder:
  python inference.py IMAGE.jpg --model . --evidence-dir evidence
Show early candidates while retaining the confirmation threshold:
  python inference.py IMAGE.jpg --model . --observation-conf 0.35 --evidence-dir evidence

Instantiate Detector once and call predict on original upright BGR frames.
Optional observation_conf=.35 exposes candidates; detector.conf stays{r['confidence']}.
Use ArrowConfirmation(conf=detector.conf,
 min_arrow_input_side=detector.min_arrow_input_side) on the current obstacle ROI,
with distinct consecutive live frame IDs. Reset for each new goal obstacle.
The helper requires3associated frames and minimum12pixels across the shorter
arrow dimension in both source frame and network input. Use original camera
frames at least640pixels wide; external pre-upscaling cannot restore detail.
Bullseye never produces a turn. This package does not drive or stop the robot.

Raw inputs, separate annotated images, JSON and contact_sheet.jpg are saved.
Capture one representative confirmed frame for each goal obstacle.

Limits: test results are in validation.json. On the supplied overhead arena
reference photo, both arrows score only~.48: available as early candidates,
but unconfirmed at{r['confidence']}. No real Pi camera distance/latency test was available.
Synthetic pixel-size success is not a measured detection range in meters.
Host timing includes preprocessing/inference/postprocessing, not camera capture.
The smoke_test_640.json file refers specifically to the640export.
''')
  previous=json.loads((folder/'manifest.json').read_text())
  previous.update(source_checkpoint_sha256=sha(ROOT/'weights/best.pt'),selected_training_epoch=r['selected_checkpoint'].get('selected_training_epoch'),confidence=r['confidence'],candidate_only=True,requires_live_pi_validation=True,files={p.name:sha(p) for p in sorted(folder.iterdir()) if p.is_file() and p.name!='manifest.json'})
  (folder/'manifest.json').write_text(json.dumps(previous,indent=2)+'\n')
  archive=folder.with_suffix('.zip')
  with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
   for p in sorted(folder.iterdir()):
    if p.is_file():z.write(p,arcname=f'{folder.name}/{p.name}')
  with zipfile.ZipFile(archive) as z:
   assert z.testzip() is None
   for name,digest in previous['files'].items():assert hashlib.sha256(z.read(f'{folder.name}/{name}')).hexdigest()==digest
  assert sha(folder/'inference.py')==sha(ROOT/'scripts/inference.py')
  summaries.append({'archive':str(archive),'bytes':archive.stat().st_size,'sha256':sha(archive),'all_member_hashes_verified':True})
 (ROOT/'reports/package_verification.json').write_text(json.dumps({'checkpoint':str(ROOT/'weights/best.pt'),'checkpoint_sha256':sha(ROOT/'weights/best.pt'),'packages':summaries},indent=2)+'\n')
 print(json.dumps(summaries,indent=2))
if __name__=='__main__':main()
