# COSPAIR
This project implements the RGB-D feature extractor, CoSPAIR  - "Colored Histograms of Spatial Concentric Surflet-Pairs for 3D object recognition" [1]

[1] Logoglu, K. Berker, Sinan Kalkan, and Alptekin Temizel. "CoSPAIR: Colored Histograms of Spatial Concentric Surflet-Pairs for 3D Object Recognition." Robotics and Autonomous Systems 75 (2016): 558-570.

**Required Librariries:**
- Opencv 3+
- PCL 1.7.2 (not tested with 1.8)

**How to build:**
cmake
make

**Usage:**
./COSPAIR   'file name'  'Support Radius (in cm)'  '# of Levels'   '# of histogram bins for depth'   'Color Space type'   '# of Color histogram bins'  'Keypoint Type? (0=uniform,1=h3d,2=iss)'   'sampling distance(if chosen)'

Example: ./COSPAIR pcd_list.txt 0.05 7 9 5 9 2 0.01

**Note: **  The input file should contain the list of "pcd" s that CoSPAIR should be extracted. The pcds should be segmented.  

Color Space Types:
1: RGB
2: RGB - L1
3: HSV
4: HSV-L1
5: CIELab (recommened and used in paper)
6: CIELab-L1