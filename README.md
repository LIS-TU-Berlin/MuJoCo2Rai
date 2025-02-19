# Mujoco to RAI Converter  

A Python tool for converting MuJoCo XML files—including joints, meshes, and textures—into `.g` files with convex decomposition, fixed mass parameters, and vertex-wise colors, making them compatible with the RAI interface.  

## Features  
- Converts MuJoCo `.xml` files while preserving joints and mesh structures.  
- Applies convex decomposition and assigns fixed mass parameters.  
- Supports textures and vertex-wise coloring.  
- Outputs ready-to-use `.g` files for RAI.  

## Example Usage 



1. **Download the RoboCasa fixture dataset Dataset** (if working with RoboCasa fixtures).  

        wget -O fixtures.zip https://utexas.box.com/shared/static/pobhbsjyacahg2mx8x4rm5fkz3wlmyzp.zip

2. Unzip the `fixture` directory inside this repository. 
       
        unzip fixtures.zip -d ./


3. Run the conversion script:  

   
        cd MuJoCo2Rai; python3 robocasa2rai.py
   

This will generate jointed RAI `.g` files in the correct format.