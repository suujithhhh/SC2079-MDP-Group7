"""Export a regression-checked circle candidate separately and compare NCNN runners."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from train_circle import ROOT, evaluate, gates, slice_metrics
import ncnn
import torch
from ultralytics import YOLO


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",default="yolo26s_seg_mdp_circle_v1")
    parser.add_argument("--data-dir",type=Path,default=ROOT/"circle_data_v1")
    args=parser.parse_args()
    review=ROOT/"runs"/(args.name+"_review")
    comparison=json.loads((review/"comparison.json").read_text())
    if not comparison["selected"]:
        raise SystemExit("No checkpoint passed the regression gates; export skipped.")
    run=Path(comparison["run"])
    candidate=run/"weights/candidate.pt"
    exported=Path(YOLO(str(candidate)).export(format="ncnn",imgsz=640,batch=1,device="cpu",half=False))
    original_net=ncnn.Net

    def four_thread_net(*a,**kw):
        net=original_net(*a,**kw)
        net.opt.num_threads=4
        return net

    ncnn.Net=four_thread_net
    torch.set_num_threads(4)
    paths=[Path(p) for p in (args.data_dir/"selection_val.txt").read_text().splitlines()]
    baseline_path=review/"baseline_ncnn.json"
    if baseline_path.exists():
        baseline=json.loads(baseline_path.read_text())
        if [r["path"] for r in baseline["records"]] != [str(p) for p in paths]:
            raise ValueError("Cached NCNN baseline belongs to another dataset")
    else:
        baseline=evaluate(ROOT/"handoff/mdp_lowres160_ncnn_model",paths,baseline_path,"cpu",1)
    result=evaluate(exported,paths,review/"candidate_ncnn.json","cpu",1)
    check=gates(baseline,result)
    report={"baseline":slice_metrics(baseline),"candidate":check,
            "export":str(exported),"production_model_replaced":False,
            "status":"candidate_for_pi_validation" if check["passed"] else "ncnn_regression_gate_failed"}
    # The original test split is evaluated only after checkpoint selection.
    if check["passed"]:
        test_paths=sorted((ROOT/"test/images").glob("*.jpg"))
        test_old=evaluate(ROOT/"handoff/mdp_lowres160_ncnn_model",test_paths,review/"test_baseline_ncnn.json","cpu",1)
        test_new=evaluate(exported,test_paths,review/"test_candidate_ncnn.json","cpu",1)
        report["original_test"]={"baseline":test_old["per_class"],"candidate":test_new["per_class"],
                                  "images":len(test_paths),"used_for_checkpoint_selection":False}
    (review/"ncnn_comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    if check["passed"]:
        handoff=ROOT/"handoff"/(args.name.removeprefix("yolo26s_seg_")+"_ncnn_model")
        if handoff.exists():
            raise FileExistsError(handoff)
        handoff.mkdir()
        for name in ("model.ncnn.param","model.ncnn.bin","metadata.yaml"):
            shutil.copy2(exported/name,handoff/name)
        for name in ("inference.py", "requirements.txt", "model_ncnn.py"):
            shutil.copy2(ROOT/"handoff/mdp_lowres160_ncnn_model"/name,handoff/name)
        shutil.copy2(review/"ncnn_comparison.json",handoff/"validation.json")
        manifest={"source_checkpoint":str(candidate),"network_imgsz":640,
                  "circle_image_id":"40","six_image_id":"16","candidate_only":True,
                  "requires_real_pi_validation":True,
                  "sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in handoff.iterdir()}}
        (handoff/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        (handoff/"README.txt").write_text(
            "Circle/6 fine-tuning candidate. Existing lowres160 handoff is unchanged.\n"
            "Use network imgsz=640, RGB normalization and the existing NCNN postprocessing.\n"
            "Circle is image ID 40 (class index 29); digit 6 is ID 16 (class index 5).\n"
            "Validation uses original validation scenes and synthetic patterned cards.\n"
            "It does NOT establish accuracy on independent Pi captures. Test actual failing\n"
            "circle photos and all other symbols before replacing the production model.\n"
            "Run: python inference.py image.jpg --conf 0.35\n")
    print(json.dumps({k:v for k,v in report.items() if k not in ("baseline","candidate")}),flush=True)


if __name__=="__main__":
    main()
