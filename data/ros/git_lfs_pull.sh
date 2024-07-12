function lfs_pull(){
    if [ -z ${1} ]; then echo "please provide directory name";return; fi;
    cur_dir=$(pwd)
    { cd ${1} 
    git lfs install
    git lfs pull
    } || {  }
    cd ${cur_dir}
}
