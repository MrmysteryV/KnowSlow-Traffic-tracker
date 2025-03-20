# KnowSlow-Traffic-tracker

Code for a Raspberry Pi traffic tracker and status reporter

Libraries used:
pip install cmake
pip install dlib
pip install numpy
pip install opencv-python

Parts of the code are inspired by https://github.com/kraten/vehicle-speed-check and the identification model is the same as the one used in that project

To run this yourself:
0. Mount and connect the Pi active cooler, the Pi camera and the ai accelerator to the pi 5
1. Download the main code, the "myhaar.xml" file and the "cars.mp4"
2. Create a virtual environment in your raspberry pi using "python -m venv /path/to/new/virtual/environment"
3. Run the virtual environment using "source /path/to/new/virtual/environment/bin/activate"
4. Install all of the modules listed above
5. cd into the folder with all of the files you downloaded
6. run "python3 Main_Code.py"
