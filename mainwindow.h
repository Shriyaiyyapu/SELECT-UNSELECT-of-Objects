#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QtOpenGL/QOpenGLBuffer>
#include <QtOpenGL/QOpenGLVertexArrayObject>
#include <QtOpenGL/QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QTreeWidget>
#include <QPlainTextEdit>
#include <QDockWidget>
#include <QFileInfo>
#include <QVector>
#include <QPair>
#include <QMap>
#include <QRegularExpression>
#include <QCheckBox>
#include "viewportobject.h"

#ifdef USE_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// Structure to hold point cloud data with rendering properties
struct PointCloud {
    QVector<QVector3D> points;
    QVector<QVector3D> colors;
    QString sourceFormat;
    bool isVisible = true;
    float pointSize = 3.0f;
    QColor tintColor = QColor(255, 255, 255);

    QVector<QVector3D> vertices;
    QVector<int> indices;
    QVector<QVector<QVector3D>> polygons;
    QVector<QVector<QVector3D>> polygonColors;
    QVector<QPair<QVector3D, QVector3D>> lines;

    QVector3D boundingBoxMin;
    QVector3D boundingBoxMax;
};

// Custom OpenGL Widget for rendering point clouds
class PointCloudGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PointCloudGLWidget(QWidget *parent = nullptr);
    ~PointCloudGLWidget();

    void setPointClouds(const QMap<QString, PointCloud>& pointClouds);
    void updatePointCloudVisibility(const QString& name, bool visible);
    void resetView();
    void setPointSize(float size);

    enum RenderMode {
        POINTS,
        POINTS_SMOOTH
    };

    void setRenderMode(RenderMode mode);
    void loadMesh(const QVector<QVector3D>& vertices, const QVector<unsigned int>& indices);

    void renderPointCloud(const PointCloud& pc);

    QString m_selectedEntityForBoundingBox;

    void showBoundingBox(const QVector3D& minCorner, const QVector3D& maxCorner);
    void hideBoundingBox();
    bool m_showBoundingBox = false;

    // Getters and setters for viewport parameters
    QMatrix4x4 getModelMatrix() const { return m_model; }
    void setModelMatrix(const QMatrix4x4& matrix) { m_model = matrix; update(); }

    QMatrix4x4 getViewMatrix() const { return m_view; }
    void setViewMatrix(const QMatrix4x4& matrix) { m_view = matrix; update(); }

    float getCameraDistance() const { return m_distance; }
    void setCameraDistance(float distance) { m_distance = distance; update(); }

    float getXRotation() const { return m_xRot; }
    void setXRotation(float xRot) { m_xRot = xRot; update(); }

    float getYRotation() const { return m_yRot; }
    void setYRotation(float yRot) { m_yRot = yRot; update(); }

    // New getters and setters for intrinsic parameters
    float getFocalDistance() const { return m_focalDistance; }
    void setFocalDistance(float focalDistance) { m_focalDistance = focalDistance; update(); }

    float getFOV() const { return m_fov; }
    void setFOV(float fov) { m_fov = fov; update(); }

    QMap<QString, PointCloud> getPointClouds() const { return m_pointClouds; }

    void setFocusOnPointCloud(const QString& name, const QVector3D& min, const QVector3D& max);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void initShaders();

    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLShaderProgram *m_program;

    QMatrix4x4 m_projection;
    QMatrix4x4 m_view;
    QMatrix4x4 m_model;

    QPoint m_lastPos;
    float m_distance;
    float m_xRot, m_yRot;
    float m_focalDistance = 0.75f; // Default focal distance
    float m_fov = 60.0f;           // Default FOV

    bool m_hasPointCloud;
    int m_pointCount;
    float m_pointSize;
    RenderMode m_renderMode;

    QMap<QString, PointCloud> m_pointClouds;

    QVector<QVector3D> m_meshVertices;
    QVector<unsigned int> m_meshIndices;
    bool m_hasMesh = false;

    void calculateSceneExtents(QVector3D& min, QVector3D& max);
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Get the currently selected point cloud
    QString getSelectedPointCloud() const;

private slots:
    void openFile();
    void resetView();
    void onItemChanged(QTreeWidgetItem *item, int column);
    void onItemClicked(QTreeWidgetItem *item, int column);
    void onItemDoubleClicked(QTreeWidgetItem *item, int column);
    void showAbout();
    void exportPointCloud();
    void showPointCloudProperties();
    void setAllVisible(bool visible);
    void saveViewportForSelectedEntity();

private:
    Ui::MainWindow *ui;
    PointCloudGLWidget *m_glWidget;
    QTreeWidget *m_treeWidget;
    QPlainTextEdit *m_textEdit;

    QMap<QString, PointCloud> m_pointClouds;
    QList<ViewportObject*> m_viewportList;
    static unsigned s_viewportIndex;

    bool loadPointCloud(const QString &filename);
    void displayPointCloudInfo(const QString &name, const PointCloud &pc);
    void setupUI();
    void createMenus();
    void updateAllVisiblePointClouds();
    void addViewportToDB(ViewportObject* viewport, const QString& entityName);
    void updateTreeWidget(ViewportObject* viewport, const QString& entityName);

    void focusCameraOnPointCloud(const QString &name);

    QString getSupportedFormatsFilter() const;

#ifdef USE_ASSIMP
    bool loadModelWithAssimp(const QString &filename);
#endif

    bool saveAsPts(const QString &filename, const PointCloud &pc);
};

#endif // MAINWINDOW_H
