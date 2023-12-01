# ws23-door-opening
 Open a door using Kinova arm
 
## Setup
- Create workspace in local machine
   ```
   mkdir ~/door_opening
   cd ~/door_opening
   
   mkdir src
   mkdir build
   ```
- Clone repository
   ```
   cd ~/door_opening/src
   git clone https://github.com/HBRS-SDP/ws23-door-opening.git
   ```
- Download kinova_api
   ```
   wget https://artifactory.kinovaapps.com/ui/native/generic-public/kortex/API/2.6.0/linux_x86-64_x86_gcc.zip
   unzip file.zip
   ```
- Build file 
   ```
   cd ~/door_opening/build
   cmake ../src/
   cmake --build
   ```
- Run
   ```
   cd ~/door_opening/app/
   ./filename
   ```
     
