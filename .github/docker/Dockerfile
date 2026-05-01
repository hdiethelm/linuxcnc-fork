#Dockerfile to create linuxCNC build container
#To run it locally:
#docker build --build-arg BASE_IMAGE=debian:trixie -f .github/docker/Dockerfile -t build-container .

ARG BASE_IMAGE

FROM $BASE_IMAGE

#Install base dependencys
RUN apt-get update
RUN apt-get install -y devscripts equivs build-essential lintian clang git lsb-release python3 gpg sudo xauth --no-install-recommends && \
     apt-get clean

#Install linuxcnc dependencys
RUN --mount=type=bind,source=.,target=/linuxcnc-src,rw \
     cd linuxcnc-src && \
     debian/configure && \
     apt-get build-dep -y . && \
     apt-get clean

#Add a user for linuxcnc with sudo nopasswd
RUN useradd -ms /bin/bash linuxcnc && \
    echo "linuxcnc ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/linuxcnc-nopasswd