#!/bin/bash

#cd ../
#xterm -hold -e "./roscore.sh"

echo im good
#sleep 5



cd
./webots/webots --mode=run /home/$USER/underdasea/worlds/worldhull1_1propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name1.txt"); catch; end; quit'

#cd 
./webots/webots --mode=run /home/$USER/underdasea/worlds/worldhull1_2propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name2.txt"); catch; end; quit'

#cd
#./webots/webots --mode=fast /home/$USER/underdasea/worlds/worldhull2_1propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name1.txt"); catch; end; quit'

#cd 
#./webots/webots --mode=fast /home/$USER/underdasea/worlds/worldhull2_2propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name2.txt"); catch; end; quit'

#cd
#./webots/webots --mode=fast /home/$USER/underdasea/worlds/worldhull3_1propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name1.txt"); catch; end; quit'

#cd 
#./webots/webots --mode=fast /home/$USER/underdasea/worlds/worldhull3_2propeller_boundary_onside.wbt

#cd 
#cd Documents/MATLAB/
#matlab -r 'try evaluationFunction("name2.txt"); catch; end; quit'


cd
cd Documents/my_test_project/refList
python3 resetNumber.py





echo I JUST FINISHED!!!!!!

