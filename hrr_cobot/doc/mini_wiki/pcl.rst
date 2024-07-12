.. _PCLDoc:

`PCL <https://pointclouds.org/>`_ support in python
****************************************************

Install via conda

.. code-block:: bash

    conda install -c conda-forge pcl

See `anaconda.org <https://anaconda.org/conda-forge/pcl>`_.

PCL workaround for broken conda-support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As a workaround you can run

.. code-block:: bash

    sudo apt install -y libpcl-dev python3-pcl

check if the library exits at

.. code-block:: bash

    cd lib/python3.8/site-packages
    ln -s | grep pcl

should print a pcl directory. Rhen link it to your conda environment, e.g.

.. code-block:: bash

    cd ~/bin/anaconda3/envs/hrr/lib/python3/dist-packages
    ln -s /usr/lib/python3/dist-packages/pcl

You may happen to adjust the packages slightly depending on your package structure
Finally test if the package is available via

.. code-block:: bash

    conda activate hrr
    python -c "import pcl"

which should not throw an error :

.. code-block:: bash

    python -c "import pcl"
    Traceback (most recent call last):
    File "<string>", line 1, in <module>
    ModuleNotFoundError: No module named 'pcl'

vs.

.. code-block:: bash

    conda activate hrr;
    python -c "import pcl"
