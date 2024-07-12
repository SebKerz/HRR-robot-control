.. _HRR_SYS_SETUP:

System Setup / control-PC configuration
------------------------------------------

In order to account for real-time capable TCP-communication
with the robot, the system should be set up via a Linux PREEMPT-RT Kernel patch.
As this, exceeds the scope of this repo, the reader is forwarded to
`<https://gitlab.lrz.de/lsr-itr-sysadmin/rt_kernel_setup>`_.

The current setup has been tested on

* Ubuntu 20.04 LTS with RT kernel patch from above
* ROS-noetic
* Anaconda with Python3.8

hostnames and usernames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

    This setup assumes you to be in the same network, thus connected to an ETH-cable in our network.
    Using eduroam and VPNs won't help

==============================  =============
hostname                        username
==============================  =============
hrrN3511rt2004.lsr.ei.tum.de    schrottgott
hrrcobotLinux43.lsr.ei.tum.de   hrr_cobot
==============================  =============

Passwords are not shown here for a reason.


Connect ot PC for first time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use `ssh-copy-id <https://www.ssh.com/academy/ssh/copy-id>`_, with your `generated ssh-key <https://www.ssh.com/academy/ssh/copy-id#generate-an-ssh-key>`_
to connect to the PC from above, e.g.:

.. code-block:: bash

    ssh-copy-id schrottgott@hrrN3511rt2004.lsr.ei.tum.de

From now on you can simply connect to the robot client via

.. code-block:: bash

    ssh schrottgott@hrrN3511rt2004.lsr.ei.tum.de

In order to setup the dedicated ROS-communication / setup, please refer to ref:`HRR_ENV_SETUP`.

Prepare System environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Installations
""""""""""""""""""""

This git expexts to have access to GIT-LFS, thus
install as outlined on `<https://git-lfs.github.com>`_,
e.g. for ubuntu 20.04 copy from `<https://packagecloud.io/github/git-lfs/install>`_ -> deb

.. code-block:: bash

    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

and then run

.. code-block:: bash

    sudo apt install git-lfs


System helpers and functions
"""""""""""""""""""""""""""""""""""

For first time usage, install ROS according to instructions online, then follow the steps outlined in the ``CI`` config

#. source access-tokens from this repo, e.g.

   .. include:: ros/source_tokens.sh
    :code: bash
    :start-line: 0

#. add function handle for git-lfs pull

   .. include:: ros/git_lfs_pull.sh
    :code: bash
    :start-line: 0

#. clone gits helper.

   .. include:: ros/clone_gits.sh
    :code: bash
    :start-line: 0

  .. note::
    Note that you should remove the function or repo ``hr_recycler_msgs`` package,
    in case you already have the ``hr_recycer`` main repo checked out at your ROS-workspace.


.. _HRR_REMOTE_ACCESS:

Enable remote access
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Quickest possibility is given by downloading the
teamviewer-**host** from https://www.teamviewer.com/en/download/linux/

Namely, download the driver from https://download.teamviewer.com/download/linux/teamviewer-host_amd64.deb
and install it on your system:

.. code-block:: bash

    wget https://download.teamviewer.com/download/linux/teamviewer-host_amd64.deb
    sudo apt install ./teamviewer-host_amd64.deb


Once installed, you have to enable teamviewer host

.. code-block:: bash

    sudo teamviewer setup

which will return

.. code-block:: text

    Welcome to TeamViewer configuration.


    Users in the Republic of Korea must consent to additional terms before using the product.
    Are you a resident of the Republic of Korea? (y/n) y

    TeamViewer® End-User License Agreement


    This End-user License Agreement including its Annex ("EULA") applies to you and TeamViewer Germany GmbH ("TeamViewer" or "We") for the licensing and use of our software, which includes the TeamViewer software and all versions, features, applications and modules thereto ("Software"). This EULA also covers any
    associated media, printed materials and electronic documentation that we make available to you (with our Software and "Product"). Future releases of our Product may warrant amendments to this EULA.

    BY CLICKING "ACCEPT", DOWNLOADING OR OTHERWISE USING OUR SOFTWARE, YOU AGREE TO ALL TERMS AND CONDITIONS OF THIS EULA. IF YOU DO NOT AGREE TO ANY OF THE TERMS OF THIS EULA, PLEASE IMMEDIATELY RETURN, DELETE OR DESTROY ALL COPIES OF OUR SOFTWARE IN YOUR POSSESSION.

    You can review the full license agreement at http://www.teamviewer.com/link/?url=271351



    Accept License Agreement? (y/n) y

    I consent to the collection and processing of my personal data as described in the Privacy Policy.
    Please choose: (y/n) y

    I consent to the international transfer of my personal data as described in the Privacy Policy.
    Please choose: (y/n) y

    I consent to the processing and use of my personal data for marketing purposes as described in the Privacy Policy.
    Please choose: (y/n) y


    This short guide helps you to setup TeamViewer on this device. After you have successfully finished the setup, this device will automatically be available in your Computers & Contacts.




    Connected
    Please enter your e-mail / username: hrc_group@lsr.ei.tum.de

    Please enter your password:
    Initiating sign in

    To ensure the continued security of your account, you need to first confirm this device is a trusted device.
    We have sent you a confirmation email containing a device authorization link. If you don't receive this verification email within a reasonable amount of time, please check your junk or spam folder.

    Connected
    Please enter your e-mail / username: hrc_group@lsr.ei.tum.de

    Please enter your password:
    Initiating sign in

    Adding this device as 'hrrcobotLinux54' to the group 'My computers' of your account HRC-Group (TUM-LSR). Do you want to continue? (y/n) [n]  y

    Adding device to your account

    Setting up your device

    *** You have successfully added this device to your Computers & Contacts. You can now access it with a simple double click in your Computers & Contacts list. ***

Alternatively, you can use the GUI, which is self-explanatory.
The user is hrc_group@lsr.ei.tum.de for this project.
Password is again not shared here for a reason.

References:

- https://community.teamviewer.com/English/kb/articles/4352-install-teamviewer-on-linux-without-graphical-user-interface

Eventually, you need to start the teamviewer instance.
Either via terminal / ssh

.. image:: ./media/teamviewer_start.png

or by opening the teamviewer-host app after logging in with the dedicated user.


.. _HRR_ENV_SETUP:

Set up ROS workspace & build project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The text below outlines how you can setup a new cobot-client
or set up your own PC / latopt to be used with the ``hrr_cobot``.
The next section outlines installation and base setups,
that should only be run once.

In the following, we assume the following variables are set

* ``$HOME`` defines the current user's home-directory.
* ``CI_SOURCE_PATH`` defines the path to where this repo is cloned to.
* ``CATKIN_WS`` defines the path toyour catkin-workspace.

An example is shown below:

.. code-block:: bash

        export HOME="/home/hrr_cobot"
        export CATKIN_WS="${HOME}/_ros/hr_recycler_ws
        export CI_SOURCE_PATH="${HOME}/_git_dev/hrr_cobot

.. note:: we recommend to add the variable-definitions to your ``~/.bashrc`` or ``~/.zshrc``

System Preparation / Installation (one time only)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""

These steps are (usually) just to be run once and can be skipped from the second connection / session on.

#. setup workspace via

   .. code-block:: bash

        sudo apt-get update -qq && apt-get -qq install wget build-essential
        if [ ! -d ${CATKIN_WS} ]; then rm -rf ${CATKIN_WS}; fi
        mkdir -p ${CATKIN_WS}/src
        ln -s $CI_SOURCE_PATH ${CATKIN_WS}/src
        echo "current workspace source:" && ls -l ${CATKIN_WS}/src


#. add internal and external dependencies.
   This will also install all dependencies for each package in your workspace

   .. code-block:: bash

        sudo apt-get -qq install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-ros-control git
        rosdep update
        source /opt/ros/$ROS_DISTRO/setup.bash
        cd ${CATKIN_WS}/src
        clone_gits
        echo "workspace source after git clone:" && ls -l ${CATKIN_WS}/src
        if [ ! -f .rosinstall ]; then wstool init; else echo "rosinstall file already there";fi
        wstool up
        cd ${CATKIN_WS}
        rosdep install --from-paths src --ignore-src -r -y

   where the variables are to be found in the ``CI`` config of this repo.

#. install Anaconda

   .. code-block:: bash

        sudo apt-get update -qq && apt-get -qq install wget build-essential
        wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
        bash ~/miniconda.sh -b -p $HOME/miniconda
        eval "$(${HOME}/miniconda/bin/conda shell.bash hook)"
        conda init

#. setup anaconda environment and activate

   .. code-block:: bash

        cd ${CI_SOURCE_PATH}/data
        conda env create -qf hrr_env_with_docu.yaml
        pip install --ignore-installed git+https://${CI_USER}:${SIM_ROBOTS_CI_TOKEN}@gitlab.lrz.de/hr_recycler/sim_robots
        conda activate hrr


#. install catkin build

   .. code-block:: bash

        sudo apt-get install -qq python3-pip
        sudo apt-get -qq install python3-catkin-tools && python3 -m pip install --upgrade osrf-pycommon

   for noetic, and

   .. code-block:: bash

        sudo apt-get -qq install python-catkin-tools

   for melodic.


.. note::

    For additional info on anaconda, please refer to the `official docu <https://docs.anaconda.com/anacondaorg/user-guide/tasks/work-with-environments/>`_.


Configure your own environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::

    The support for these helper functions is limited to ``zsh``,
    you can copy the available functions in your bashrc and they are expected to work.
    Nonetheless, use at your own risk.


The scripts outlined above can be quickly added to your current environment,
you may still need to adjust the variables to your local setup:

* ``HOME`` as above
* ``HRR_CATKIN_WS`` as the catkin-workspace of the project, we use ``${HOME}/_ros/hr_recycler_ws`` in the following
* ``HRR_REPO_PATH`` as the path to this repository on the current PC.
* ``HR_RECYCLER_IP``: is optional and sets the IP of the **current PC**, by default the LSR-IP is used. For external application, copy the value of the IP in there.
* ``HRR_ROS_MASTER``: describes the **hostname** of the rosmaster, i.e. the client, where the roscore will be running on later

.. warning::

    For the pilot setups, please adjust

    * ``HRR_ROS_MASTER`` as the pilot ROS-master.
    * ``HR_RECYCLER_IP`` as the current IP, hopefully a static one.


We can use the ``set_shell`` helper in this repo, to adjust the environment quickly:

.. include:: scripts/set_shell.sh
    :code: bash
    :start-line: 0

Thus, simply add the following lines to your ``.zshrc`` or ``.bashrc`` (experimental):

.. code-block:: bash

    HR_RECYCLER_IP=""
    HRR_ROS_MASTER="hrrcobotLinux54.lsr.ei.tum.de"
    HRR_CATKIN_WS="${HOME}/_ros/hr_recycler_ws"
    HRR_REPO_PATH="${HOME}/_git_dev/hrr_cobot"
    KB_SCRIPT_PATH="${HRR_REPO_PATH}/data"
    { source ${KB_SCRIPT_PATH}/set_shell.sh }||{ echo "failed to load hrr-cobot helper scripts" }

In case ``Anaconda`` has added some lines to your zshrc, you need to wrap them by a function called

.. code-block:: bash

    function condafy(){
        # all the stuff from conda
    }

Below you can find a sample `.zshrc` from the lab

.. code-block:: bash

    export HOME="/home/hrr_cobot"
    export ZSH="${HOME}/.oh-my-zsh"
    emulate sh -c "source /etc/profile"
    ZSH_THEME="powerlevel10k/powerlevel10k"
    plugins=(git debian pip ros k)
    source "$ZSH/oh-my-zsh.sh"
    source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh
    alias gab_commit="git commit --author='Volker Gabler <v.gabler@tum.de>'"
    export PATH="${HOME}/.local/bin:${PATH}"

    function condafy(){

        # >>> conda initialize >>>
        # !! Contents within this block are managed by 'conda init' !!
        __conda_setup="$('/home/hrr_cobot/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
        if [ $? -eq 0 ]; then
            eval "$__conda_setup"
        else
            if [ -f "/home/hrr_cobot/anaconda3/etc/profile.d/conda.sh" ]; then
            . "/home/hrr_cobot/anaconda3/etc/profile.d/conda.sh"
            else
            export PATH="/home/hrr_cobot/anaconda3/bin:$PATH"
            fi
        fi
        unset __conda_setup
        # <<< conda initialize <<<
    }

    export ROSCONSOLE_FORMAT='[${severity}]${message}'
    HR_RECYCLER_IP=""
    HRR_ROS_MASTER="hrrcobotLinux54.lsr.ei.tum.de"
    HRR_CATKIN_WS="${HOME}/_ros/hr_recycler_ws"
    HRR_REPO_PATH="${HOME}/_git_dev/hrr_cobot"
    KB_SCRIPT_PATH="${HRR_REPO_PATH}/data"
    { source ${KB_SCRIPT_PATH}/set_shell.sh }||{ echo "failed to load hrr-cobot helper scripts" }

    if [ -n ${HR_RECYCLER_IP} ]; then
        hr_recycler_rosify
    else
        hrr_rosify
    fi

    if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
    source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
    fi


    # To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
    [[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh



    for a final setup

.. warning::

    if you copy the paths above without thinking / reading, nothing will work eventually.


compile ROS workspace and run quick-test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming the system is set up as above, i.e. ``hrr_rosify`` has been called,
the ROS workspace can be build as shown below:

#. compile / build the workspace

   .. code-block:: bash

        cd ${HRR_CATKIN_WS}
        catkin build


#. test the current environment for import errors

   .. code-block:: bash

        python -c "from hrr_common import *"
        python -c "from hrr_controllers import *"
        python -c "from hrr_cobot_robot import *"
        python -c "from sim_robots import get_racer_kin_model;import numpy as np;model=get_racer_kin_model();print(model.forward_kin(np.zeros(6)))"

