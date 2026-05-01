ARG BASE_IMAGE

FROM $BASE_IMAGE

#Install base dependencys
RUN apt-get update
RUN apt-get install -y devscripts equivs build-essential lintian clang git lsb-release python3 gpg sudo xauth --no-install-recommends

#Install linuxcnc dependencys
#We need only the debian folder
COPY debian /linuxcnc-src/debian
RUN  cd linuxcnc-src && \
     debian/configure && \
     apt-get build-dep -y .

#Cleanup
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf linuxcnc-src

#Add a user for linuxcnc with sudo nopasswd and set it as user
RUN useradd -ms /bin/bash linuxcnc && \
    echo "linuxcnc ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/linuxcnc-nopasswd