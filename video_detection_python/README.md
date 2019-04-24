Training YOLOv3 Object Detector - Snowman

1. Create a folder contains images files and name it "images".

2. Create a folder contains annotations files and name it "labels". Folders "images" and "labels" must be in the same directory.

3. Download  an application `https://github.com/tzutalin/labelImg` to annotate the objects in images.

4. Open LabelImg application.

5. Click on "Open Dir" and then choose the Images folder.

6. Click on "Change Save Dir" and choose the labels folder.

7. Right below "Save" button in the toolbar, click "PascalVOC" button to switch to YOLO format.

8. You will find that all images are listed in the File List panel.

9. Click on the Image you want to annotate. Click the letter "W" from your keyboard to draw the rectangle on the desired image object, type the name of the object on the  popped up window. Click "CTRL+S" to save the annotation to the labels folder.

10. Repeat steps 9 till you complete annotating all the images.

11. Create the train-test split

`python3 splitTrainAndTest.py /path/Images`

Give the correct path to the data Images folder. The 'labels' folder should be in the same directory as the Images folder.

12. Install Darknet and compile it.
```
cd ~
git clone https://github.com/pjreddie/darknet
cd darknet
make
```
12. Preparing training configuration files

    read more at `https://www.learnopencv.com/training-yolov3-deep-learning-based-custom-object-detector/` and 
    `http://emaraic.com/blog/yolov3-custom-object-detector`

13. Get the pretrained model

`wget https://pjreddie.com/media/files/darknet53.conv.74 -O ~/darknet/darknet53.conv.74`

14. Fill in correct paths in the trainer.data file

15. Start the training as below, by giving the correct paths to all the files being used as arguments

`cd ~/darknet`

`./darknet detector train /path/trainer.data  /path/yolov3-tiny.cfg ./darknet53.conv.74 > /path/train.log`

16. Give the correct path to the modelConfiguration and modelWeights files in object_detection_yolo.py and test any image or video.

`python3 object_detection_yolo.py --image=/path/image.jpg`

