if [ "$1" == "rb" ]; then 
    echo "Rebuilding all ..."
    rm -rf build/ devel/
fi
catkin_make
source devel/setup.bash