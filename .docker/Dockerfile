FROM espressif/idf

ARG USERNAME=esp
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get -y update && \
    apt-get -y upgrade && \
    apt-get -y install sudo &&\
    apt-get -y install nano 

########################################################################################
# Add our non-root user (host mirror) and give them sudo priviledges 
########################################################################################
RUN userdel -r $(getent passwd $USER_UID | cut -d: -f1) 2>/dev/null || true && \
    groupdel $(getent group $USER_GID | cut -d: -f1) 2>/dev/null || true && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -ms /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod -aG root $USERNAME && \
    usermod -aG dialout $USERNAME

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USERNAME

RUN echo ". /opt/esp/idf/export.sh > /dev/null 2>&1" >> /home/$USERNAME/.bashrc && \
    echo "export LC_ALL=C" >> /home/$USERNAME/.bashrc




    




    
    