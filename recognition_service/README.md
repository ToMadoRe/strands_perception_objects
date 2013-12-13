Call the recognition_service by:

``` rosrun recognition_service recognition_service -models_dir /home/thomas/willow_dataset/models/ -training_dir_sift /home/thomas/willow_dataset/sift_trained/ -recognizer_structure_sift /home/thomas/willow_dataset/willow_structure/ -do_sift 1 -do_ourcvfh 0 -training_dir_ourcvfh /home/thomas/willow_dataset/ourcvfh_trained/ -chop_z 1.2 -icp_iterations 10```

where ```-models_dir``` specifies the directory containing the models, ```-recognizer_structure``` sift is the required structure of the models, ```-chop_z``` is used as maximum z-distance (depth) of the point cloud for the pass-through filter, the boolean parameters ```-do_sift``` and ```-do_ourcvfh``` specify the method used for recognition (SIFT or OURCVFH), ```-icp_iterations``` specifies the maximum iterations of icp.
The training directories ``` -training_dir_sift``` and ```-training_dir_ourcvfh``` are filled with training data during initialization phase and don't have to be create beforehand. 

During initialization the programm creates a file ```sift_flann.idx``` in the current working directory, which has to be deleted after the model directory (and/or structure dir) has been updated. 
