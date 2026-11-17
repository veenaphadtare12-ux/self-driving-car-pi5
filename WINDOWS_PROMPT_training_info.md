I have a graded project review tomorrow: a self-driving model car (Raspberry Pi 5) that follows lanes with a UFLD model and detects obstacles, traffic lights and stop signs with a YOLO model. The YOLO model was trained on THIS Windows PC. I need every training and accuracy detail for my PowerPoint, collected into one folder that I will copy to the Pi.

RULES: work READ-ONLY. Do not retrain, do not delete, move or edit any existing file. Running `yolo val` and copying files into the new output folder is fine. If something is not found, write "not found" and say where you looked; do not guess numbers.

## What I already know about the deployed model (from its metadata on the Pi)
- Ultralytics YOLO26n, Ultralytics 8.4.157, trained 2026-11-14 around 01:45 IST
- Trained with: C:\Users\Veena\self_driving_car_training\data\own_track_images\obstacles\roboflow_export\data.yaml
- imgsz 416, 6 classes: 0 animal, 1 car, 2 light_green, 3 light_red, 4 person, 5 stop_sign
- Exported to NCNN and runs on the Pi as `obstacle_v1_ncnn_model`
- On the Pi it is used with: confidence 0.40 for person/car/animal, 0.50 for traffic lights and stop sign;
  a red light must be seen in 3 YOLO checks in a row before the car stops.

## Output
Create `C:\Users\Veena\self_driving_car_training\ppt_training_info\` with this layout:
```
ppt_training_info\
  TRAINING_SUMMARY.md      <- the main report (sections A-J below)
  plots\                   <- training/validation plots
  samples\                 <- a few example images
  model\                   <- best.pt, last.pt (if present), args.yaml, results.csv, data.yaml, export files info
  extra\                   <- anything else useful (other runs' results.csv, Roboflow README, UFLD files)
```
At the end, also make `ppt_training_info.zip` of that folder.

## A. Find the training runs
1. Search C:\Users\Veena (especially self_driving_car_training) for Ultralytics run folders (folders with `args.yaml` + `results.csv` + `weights\`).
2. Make a table of ALL YOLO runs: folder, date, model (yolo26n/yolov8n/...), dataset yaml, epochs run, best mAP50, best mAP50-95.
3. Say which run is the deployed one (2026-11-14, YOLO26n, 6 classes, imgsz 416; compare the best.pt date with the export/ NCNN folder if it exists). Use that run for everything below.
4. The run history shows how the model improved; keep it as "Experiments" in the summary.

## B. Training setup (from args.yaml and results.csv of the deployed run)
- starting weights (e.g. yolo26n.pt pretrained on COCO) - i.e. transfer learning or from scratch
- epochs planned vs actually run, early stopping / patience, best epoch
- imgsz, batch, optimizer, lr0, lrf, momentum, weight decay, warmup
- augmentation: mosaic, mixup, hsv_h/s/v, fliplr, flipud, degrees, translate, scale, close_mosaic
- hardware and software: GPU model (or CPU), CUDA, Python, torch and ultralytics versions (`yolo checks`, `nvidia-smi`)
- total training time (sum of epoch times or first/last file timestamps)
- model size: parameters, GFLOPs, best.pt file size

## C. Dataset
- number of images in train / valid / test, and the split ratio
- number of labelled boxes per class in each split (count lines in the YOLO label .txt files)
- images with no objects (background images), if any
- where the images came from: photos of my own track and toys; Roboflow project/version name, preprocessing and augmentation (read the README.dataset.txt / README.roboflow.txt files in the export)
- class names in data.yaml exactly as written (note: an older export on the Pi used names like person, dog, toy_car - say if the Windows data.yaml differs)
- copy 2 example training images per class (with their boxes drawn if easy, e.g. from train_batch*.jpg) into samples\

## D. Accuracy (validation)
Run on the deployed best.pt:
```
yolo val model=<run>\weights\best.pt data=<data.yaml> imgsz=416 split=val plots=True project=<ppt_training_info>\extra name=val_run
```
(also `split=test` if a test split exists). Report:
- overall: Precision, Recall, mAP50, mAP50-95
- per class: images, instances, P, R, mAP50, mAP50-95
- the speed line (preprocess / inference / postprocess ms per image) and on which GPU/CPU
- the best values from results.csv during training and at which epoch (they should match the val run; say if not)

## E. Thresholds used on the car
From the val run's P, R and F1 curves (or by re-running val with `conf=0.4` and `conf=0.5`):
- per class precision and recall at confidence 0.40 (person, car, animal) and 0.50 (light_red, light_green, stop_sign)
- the confidence where F1 is highest (from BoxF1_curve)
This shows whether the thresholds on the Pi are sensible.

## F. Safety check: red vs green
From the normalized confusion matrix (and the val predictions):
- how often light_red is predicted as light_green and the other way round (must be 0 or near 0)
- how often each class is missed (predicted as background) and how often background gives a false box
- the weakest class (lowest mAP50) and a likely reason (few examples, small objects, similar look, lighting)

## G. Plots (copy into plots\, keep the original names)
From the training run and the val run: results.png, confusion_matrix.png, confusion_matrix_normalized.png,
BoxPR_curve.png, BoxF1_curve.png, BoxP_curve.png, BoxR_curve.png, labels.jpg, train_batch0.jpg,
val_batch0_labels.jpg, val_batch0_pred.jpg (and val_batch1/2 if present).
In the summary, list each file with one line on what it shows.

## H. Export to the Raspberry Pi
- how the NCNN model was made (command / notebook / script, e.g. `yolo export model=best.pt format=ncnn imgsz=416`), if you can find it in history, scripts or notebooks
- copy into model\: best.pt (the deployed one), last.pt if present, args.yaml, results.csv, data.yaml
  (I need best.pt on the Pi to compare the NCNN model's detections with the original model)

## I. Lane model (UFLD) files - only if they are on this PC
UFLD was trained on Kaggle (notebooks ufld_finetune_v1/v2.ipynb, runs ufld_v1, ufld_v2_same, ufld_v2_plus). The Pi already has the final accuracy. If this PC has any downloaded Kaggle output, copy into extra\ufld\:
- per-epoch training loss / log (training log text, metrics csv, or the executed notebook .ipynb with outputs)
- results.json, val_overlays.jpg, comparison_v1_v2.csv
Do NOT copy .pth model files (too big, the Pi has the ONNX models). If nothing is found, say so.

## J. Existing presentation material
Search for any existing PPT/PPTX/DOCX/PDF report, architecture diagram, or slides about this project (Desktop, Documents, Downloads, self_driving_car_training). List them with path and date; copy them into extra\docs\ if under 50 MB.

## TRAINING_SUMMARY.md layout
1. Model and training setup (table, from B)
2. Experiments / run history (table, from A)
3. Dataset (tables, from C)
4. Accuracy overall + per class (tables, from D)
5. Thresholds used on the car (table, from E)
6. Red vs green safety + errors (from F)
7. Strengths and weaknesses (short list)
8. 6-8 slide-ready bullet points in simple English
9. Plot and sample file list with one line each
10. Anything not found and where you looked

At the end print the full path of ppt_training_info.zip and the tables from sections 1, 3 and 4.
