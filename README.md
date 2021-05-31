# Serial Robot Dynamics port to python
Source: https://github.com/SergeiSa/SRD

### Required packages:
+ numpy
+ casadi
+ control
+ slycot 
+ urdf-parser-py
+ pyngrok
+ pybullet
+ qpsolvers

And meshcat installed from git
```
pip install git+https://github.com/rdeits/meshcat-python.git@master
```
### Windows requirements
+ mingw-64 with clang support

### Linux requirements
```
sudo apt-get install clang-9
```



You might have some issues with slycot installation through pip. Then you have to use conda instead
```
conda install -c conda-forge slycot
```

After you are done with the environment setup just cd into the cloned repo folder and run
```
pip install ./SrdPy
```

