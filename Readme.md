# Prerequisites
## Golog++
```
cd
wget https://sourceforge.net/projects/maskor/files/gologpp.deb
sudo dpkg -i gologpp.deb
rm gologpp.deb
```
If /usr/local/lib not in $LD_LIBRARY_PATH:
```
echo '
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
. ~/.bashrc
```
## Eclipse
```
cd /opt
sudo wget https://eclipseclp.org/Distribution/Builds/7.1_13/x86_64_linux/eclipse_basic.tgz
sudo mkdir -p eclipse_basic && cd eclipse_basic
sudo tar xpfz ../eclipse_basic.tgz
sudo rm ../eclipse_basic.tgz
sudo ./RUNME # Hit all enters
```
```
echo '
export LD_LIBRARY_PATH=/usr/local/lib:/opt/eclipse_basic/lib:/opt/eclipse_basic/lib/x86_64_linux:$LD_LIBRARY_PATH
export PATH=/opt/eclipse_basic/bin/x86_64_linux:$PATH
export CPLUS_INCLUDE_PATH=/opt/eclipse_basic/include/x86_64_linux:$CPLUS_INCLUDE_PATH' >> ~/.bashrc
. ~/.bashrc
```