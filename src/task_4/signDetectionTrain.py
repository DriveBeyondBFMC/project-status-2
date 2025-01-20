#!/usr/bin/env python3
from ultralytics import YOLO
import argparse
import sys
import ast 

def parse_list(arg):
    try:
        return ast.literal_eval(arg)
    except ValueError:
        raise argparse.ArgumentTypeError("Argument must be a list of integers.")

parser = argparse.ArgumentParser()
parser.add_argument('-directory', type=str, help='Directory to model for training')
parser.add_argument('-project', type=str, help='Parent folder name')
parser.add_argument('-name', type=str, help="Name of checkpoint")
parser.add_argument('-batch', type=int, help="Number of simultaneous training images")
parser.add_argument('-epoch', type=int, help="Number of epochs")
parser.add_argument('-freeze', type=parse_list, help="Freezing range")
args = parser.parse_args()

global num_freeze
num_freeze = args.freeze

def freeze_layer(trainer):
    model = trainer.model
    print(f"Freezing {num_freeze[1] - num_freeze[0]} layers")
    freeze = [f'model.{x}.' for x in range(*num_freeze)]  # layers to freeze 
    for k, v in model.named_parameters(): 
        v.requires_grad = True  # train all layers 
        if any(x in k for x in freeze): 
            print(f'freezing {k}') 
            v.requires_grad = False 
    print(f"{num_freeze[1] - num_freeze[0]} layers are freezed.")

model = YOLO(model = args.directory, task = 'segment')
model.add_callback("on_train_start", freeze_layer)

try:
    model.train(
        data = "./dataset/data.yaml",
        epochs = args.epoch,
        imgsz = 640, 
        
        batch = args.batch, 
        patience = 1000,
        verbose = True,
        optimizer = 'auto',
        cos_lr = True, 
        lr0 = 0.001,
        momentum = .9,
        
        device = [0],
        workers = 12,
        cache = 'ram',

        plots = True,
        show = True, 
        visualize = True,
        save = True, 
        save_txt = True,
        show_boxes = True,
        
        
        project = f"{args.project}", 
        name = args.name,
        seed = 0, 
        exist_ok = True,
        
    )
     
except RuntimeError as e:
    print(f"An error occurred: {e}", file=sys.stderr)
    sys.exit(1)
sys.exit(0)