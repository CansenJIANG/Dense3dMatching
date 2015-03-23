#ifndef pclQviewer_H
#define pclQviewer_H

// include common header files and data type configuration
#include "commonFunc.h"

#include <Eigen/Core>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

/////////////////////////////////////////////////////////////////////////////////////
/// define call back structure for point clicking selection
/////////////////////////////////////////////////////////////////////////////////////
struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    // pcl::visualization::PCLVisualizer::Ptr viewerPtr;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
    // text editor output
    QPlainTextEdit *txtEditor;
    // color of keypoints
    uc8 ptColor[3];
};

/////////////////////////////////////////////////////////////////////////////////////
/// define key point detector constructor
/////////////////////////////////////////////////////////////////////////////////////
struct str_featDescr{
    // key point name in pcl viewer
    std::string name;
    // key point detector index
    uc8 detectorIdx;
    // Key point detector setting parameters
    f32 params[5];
    // key point color
    uc8 matchColor[3];
    // key point size
    uc8 viewSize;
    // match indices
    std::vector<u16> matchIdx1;
    std::vector<u16> matchIdx2;
    std::vector<f32> matchDist;
    // state of drawing lines
    uc8 lineDrawOn;
    // draw line index
    u16 lineIdx;
    // line width
    f32 lineWidth;
};

/////////////////////////////////////////////////////////////////////////////////////
/// define feature descriptor constructor
/////////////////////////////////////////////////////////////////////////////////////
struct str_keyPts{
    // key point detector name
    std::string name;
    // filtered key point name
    std::string filteredName;
    std::string filteredName2;
    // key point detector index
    uc8 detectorIdx;
    // Key point detector setting parameters
    f32 params[15];
    // key point color
    uc8 viewColor[3];
    // key point size
    uc8 viewSize;
};


namespace Ui
{
    class pclQviewer;
}

class pclQviewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit pclQviewer (QWidget *parent = 0);
    ~pclQviewer ();

public slots:
    // slider to change the size of point cloud
    void pSliderValueChanged (int value);
    // slider to move point cloud along z axis
    void movePcSlider (int value);


protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; // Point cloud viewer
    std::string fName;                  // name of the point cloud 1
    std::string fName2;                  // name of the point cloud 2
    PointCloudT::Ptr cloud;             // Point cloud of scene
    PointCloudT::Ptr cloud2;            // New point cloud of scene
    PointCloudT::Ptr keyPts;            // key points detected from the scene
    PointCloudT::Ptr keyPts2;           // key points detected from new point cloud
    PointCloudT::Ptr featurePts;        // feature points manually selected from the point cloud
    PointCloudT::Ptr featurePts2;       // feature points manually selected from the point cloud
    PointCloudT::Ptr filteredKeyPts;    // remained key points after filtering
    PointCloudT::Ptr filteredKeyPts2;   // remained key points after filtering

    // container for point cloud colors
    bool showColor;
    std::vector<uc8> cloudR;
    std::vector<uc8> cloudG;
    std::vector<uc8> cloudB;
    uc8 red;
    uc8 green;
    uc8 blue;

    // Preprocessing params
    f32 clipThd;
    f32 shiftPC_X;
    f32 shiftPC_Y;
    f32 shiftPC_Z;

    // Keypoint detector parameters structure
    struct str_keyPts keyPtsStr;

    // Feature detector parameters structure
    struct str_featDescr featDescrStr;

    // func to select feature points
    void on_getPoint_clicked();

    // func to draw key points
    void drawKeyPts(const PointCloudT::Ptr &keyPts, const std::string pcName,
                       const uc8 pColor[], const uc8 ptSize);


    // Add point picking callback to viewer:
    struct callback_args cb_args;

private slots:

    ///*******************************************
    ///* Point Cloud Loading SLOTs               *
    ///*******************************************
    // click botton to load point cloud
    void on_LoadPC_clicked();
    void on_showCloud_1_clicked();

    // click botton add multiple point cloud to viewer
    void on_add_PC_clicked();
    void on_showCloud_2_clicked();

    // check box to visualize point cloud color
    void on_chkbox_withColor_clicked();

    // click botton to point cloud voxelization
    void on_getVoxel_clicked();

    // click botton to start kinect stream
    void on_StartKinect_clicked();

    // click button to save selected features
    void on_saveFeatures_clicked();

    // click button to take a screenshot of the widget
    void on_takeScreenshot_clicked();

    // click button to delete all the selected features
    void on_cleanFeatures_clicked();

    // click button to delete the last selected feature
    void on_delOnePt_clicked();

    // click button to remove points further than threshold
    void on_clipThreshold_editingFinished();



    ///*******************************************
    ///* Preprocessing of point cloud            *
    ///*******************************************

    void on_loadSelectedFeat_clicked();

    void on_loadMatchIdx_clicked();

    void on_transformPc_clicked();

    void on_clipPC_clicked();

    void on_shiftX_val_editingFinished();
    void on_shiftY_val_editingFinished();
    void on_shiftZ_val_editingFinished();

private:
    Ui::pclQviewer *ui;

};

#endif // PCLVIEWER_H
