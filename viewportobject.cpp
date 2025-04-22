#include "viewportobject.h"
#include "mainwindow.h"
#include <QDebug>

ViewportObject::ViewportObject(const QString& name)
    : m_name(name)
{
}

ViewportObject::~ViewportObject()
{
}

QString ViewportObject::getName() const
{
    return m_name;
}

void ViewportObject::setParameters(const ViewportParameters& params)
{
    m_params = params;
}

void ViewportObject::applyViewport(PointCloudGLWidget* glWidget)
{
    if (glWidget) {
        glWidget->setModelMatrix(m_params.modelMatrix);
        glWidget->setViewMatrix(m_params.viewMatrix);
        glWidget->setCameraDistance(m_params.cameraDistance);
        glWidget->setXRotation(m_params.xRot);
        glWidget->setYRotation(m_params.yRot);

        // Apply intrinsic parameters
        glWidget->setFocalDistance(m_params.focalDistance);
        glWidget->setFOV(m_params.fov);

        glWidget->update();
    }
}
