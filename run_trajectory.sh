PROJECTPATH=/home/yutian/Downloads/TartanAir_shibuya
OUTCOME=/home/yutian/Downloads/AirDOS-dev/Evaluation/data
METAOUT=/home/yutian/Downloads/AirDOS-dev/Evaluation/meta
EXPNAME=3_result_RoadCrossing04
SEQ=RoadCrossing06
echo $PROJECTPATH/$SEQ;

#Examples/Monocular/mono_kitti \
#    Vocabulary/ORBvoc.txt \
#    ../AirDOS-dev/Examples/Stereo/config/tartanair.yaml \
#    $PROJECTPATH/$SEQ;

Examples/Monocular/mono_kitti \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/KITTI00-02.yaml \
  /home/yutian/Downloads/KITTI_VO/sequences/00;

# Examples/Stereo/stereo_kitti_human \
#    Vocabulary/ORBvoc.txt \
#    Examples/Stereo/config/tartanair.yaml \
#    $PROJECTPATH/$SEQ \
#    $OUTCOME/$EXPNAME.txt \
#    $METAOUT;
