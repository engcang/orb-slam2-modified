# Testing the Code from [Mapping_Localization-ORB-SLAM2](https://github.com/Alkaid-Benetnash/ORB_SLAM2)

<br><br><br>


### Removed Pangolin
### Removed Viewer (to reduce thread) -> merged into system and main code (e.g. stereo_map.cc)
### used speed-up vocabulary from [here](https://github.com/raulmur/ORB_SLAM2/pull/21)
### CPU affinity is added by **Local-Ryu** to allocate the threads to wanted CPU cores (System.cc and LocalMapping.cc + Frame.cc (Updated))
-> change **mask** at LocalMapping::Run() function to allocate threads to wanted CPU <br>
-> change **mask** at Frame::ExtractORB() : only for Stereo
<br><br><br><br><br>


# ORB-SLAM2 : [here](https://github.com/raulmur/ORB_SLAM2)
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))
