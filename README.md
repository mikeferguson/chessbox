## Chess for Robots

This is the n-th generation chess code, which was originally developed for AAAI 2011.

## Installation

    sudo apt-get install gnuchess gnuchess-book
    sudo apt-get install festlex-cmu ros-groovy-sound-drivers

### Setup for festival

    cd /usr/share/festival/voices/english
    sudo wget -c http://www.speech.cs.cmu.edu/cmu_arctic/packed/cmu_us_awb_arctic-0.95-release.tar.bz2
    sudo tar jxf cmu_us_awb_arctic-0.95-release.tar.bz2 
    sudo ln -s cmu_us_awb_arctic cmu_us_awb_arctic_clunits
