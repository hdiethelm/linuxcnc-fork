#Dockerfile to create linuxCNC build container
#To run it locally:
#docker build --build-arg BASE_IMAGE=debian:trixie -f .github/docker/Dockerfile -t build-container .

ARG BASE_IMAGE

FROM $BASE_IMAGE

# Set DEBIAN_FRONTEND to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

#Install base dependencys
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        devscripts equivs build-essential lintian clang git lsb-release \
        python3 gpg dirmngr sudo xauth curl kmod \
        cppcheck shellcheck && \
     apt-get clean

#Install linuxcnc build dependencys
RUN --mount=type=bind,source=.,target=/linuxcnc-src,rw \
     cd linuxcnc-src && \
     debian/configure && \
     apt-get -y build-dep . && \
     apt-get clean

#Install linuxcnc install dependencys
# Needs all debian packages: Build them
# Install them
# Remove the linuxcnc packages selectively
# Mark all autoremove packages as manual, so apt doesn't complain in later build steps
# Remove built packages
#A bit hacky but does the job
RUN --mount=type=bind,source=.,target=/linuxcnc-src,rw \
     cd linuxcnc-src && \
     debian/configure && \
     debuild -us -uc --build=any && \
     apt-get install -y --no-install-recommends \
       ../linuxcnc-uspace_*.deb \
       ../linuxcnc-uspace-dev_*.deb \
       ../linuxcnc-uspace-dbgsym_*.deb && \
     apt-get purge -y linuxcnc-uspace linuxcnc-uspace-dev linuxcnc-uspace-dbgsym && \
     apt-mark manual $(apt-get --dry-run autoremove | grep -Po '^Remv \K[^ ]+') && \
     rm -f ../linuxcnc*.* && \
     apt-get clean

#Add a user for linuxcnc with sudo nopasswd
RUN useradd -ms /bin/bash linuxcnc && \
    echo "linuxcnc ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/linuxcnc-nopasswd