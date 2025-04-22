#ifndef VIEWPORTOBJECT_H
#define VIEWPORTOBJECT_H

#include <QMatrix4x4>
#include <QVector3D>

class PointCloudGLWidget;

class ViewportObject
{
public:
    struct ViewportParameters {
        QMatrix4x4 modelMatrix;
        QMatrix4x4 viewMatrix;
        float cameraDistance;
        float xRot;
        float yRot;
        QVector3D modelCenter;
        float focalDistance; // Added for intrinsic camera parameter
        float fov;          // Added for field of view
    };

    explicit ViewportObject(const QString& name);
    ~ViewportObject();

    QString getName() const;
    void setParameters(const ViewportParameters& params);
    void applyViewport(PointCloudGLWidget* glWidget);

private:
    QString m_name;
    ViewportParameters m_params;
};

#endif // VIEWPORTOBJECT_H
