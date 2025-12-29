# Deepstream Installation on Nvidia Jetson Orin.
# Jetpack version
-Jetpack 6.0
Refer:
https://developer.nvidia.com/deepstream-sdk?utm_source=copilot.com#

dpkg -l | grep nvidia-l4t-core
ii  nvidia-l4t-core                              36.3.0-20240719161631                       arm64        NVIDIA Core Package


Install deepstream from :
https://catalog.ngc.nvidia.com/orgs/nvidia/resources/deepstream/files?version=7.1
Verison : deepstream-7.1_7.1.0-1_arm64.deb download.
Or 
wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/deepstream/7.1/files?redirect=true&path=deepstream-7.1_7.1.0-1_arm64.deb' --output-document 'deepstream-7.1_7.1.0-1_arm64.deb'
rahul@rahul-nyra:~$ sudo apt install ./deepstream-7.1_7.1.0-1_arm64.deb 
[sudo] password for rahul: 
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Note, selecting 'deepstream-7.1' instead of './deepstream-7.1_7.1.0-1_arm64.deb'
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 deepstream-7.1 : Depends: libnvinfer10 (>= 10.0.0) but it is not installable
                  Depends: libnvinfer-dev (>= 10.0.0) but 8.6.2.3-1+cuda12.2 is to be installed
                  Depends: libnvonnxparsers10 (>= 10.0.0) but it is not installable
                  Depends: libnvonnxparsers-dev (>= 10.0.0) but 8.6.2.3-1+cuda12.2 is to be installed
                  Depends: libnvinfer-plugin10 (>= 10.0.0) but it is not installable
                  Depends: libnvinfer-plugin-dev (>= 10.0.0) but 8.6.2.3-1+cuda12.2 is to be installed
E: Unable to correct problems, you have held broken packages.


So We have to downgrade deepstream.
wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/deepstream/7.0/files?redirect=true&path=deepstream-7.0_7.0.0-1_arm64.deb' --output-document 'deepstream-7.0_7.0.0-1_arm64.deb'



rahul@rahul-nyra:~$ sudo apt install ./deepstream-7.0_7.0.0-1_arm64.deb 



update-alternatives: using /opt/nvidia/deepstream/deepstream-7.0/bin/service-maker-test3-app to provide /usr/bin/service-maker-test3-app (service-maker-test3-app) in auto mode
update-alternatives: using /opt/nvidia/deepstream/deepstream-7.0/bin/service-maker-test4-app to provide /usr/bin/service-maker-test4-app (service-maker-test4-app) in auto mode
update-alternatives: using /opt/nvidia/deepstream/deepstream-7.0/bin/service-maker-test5-app to provide /usr/bin/service-maker-test5-app (service-maker-test5-app) in auto mode
---------------------------------------------------------------------------------------
NOTE: sources and samples folders will be found in /opt/nvidia/deepstream/deepstream-7.0
---------------------------------------------------------------------------------------
Processing triggers for libc-bin (2.35-0ubuntu3.11) ...


Check the pipeline works by doing this :

gst-launch-1.0 v4l2src device=/dev/video0 ! \
'video/x-raw,format=YUY2,width=1280,height=720' ! \
nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! \
nveglglessink


