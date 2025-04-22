
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QMatrix4x4>
#include <QtMath>
#include <QMouseEvent>
#include <QOpenGLShader>
#include <QProgressDialog>
#include <QCheckBox>
#include <QLabel>
#include <QSlider>
#include <QPushButton>
#include <QColorDialog>

unsigned MainWindow::s_viewportIndex = 0;

// ========== PointCloudGLWidget Implementation ==========

PointCloudGLWidget::PointCloudGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_program(nullptr)
    , m_distance(5.0f)
    , m_xRot(0.0f)
    , m_yRot(0.0f)
    , m_hasPointCloud(false)
    , m_pointCount(0)
    , m_pointSize(3.0f)
    , m_renderMode(POINTS)
{
    setFocusPolicy(Qt::StrongFocus);
}

PointCloudGLWidget::~PointCloudGLWidget()
{
    makeCurrent();
    m_vbo.destroy();
    m_vao.destroy();
    delete m_program;
    doneCurrent();
}

void PointCloudGLWidget::setPointSize(float size)
{
    m_pointSize = size;
    update();
}

void PointCloudGLWidget::setRenderMode(RenderMode mode)
{
    m_renderMode = mode;
    update();
}

void PointCloudGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    m_vao.create();
    m_vbo.create();

    initShaders();
}

void PointCloudGLWidget::initShaders()
{
    m_program = new QOpenGLShaderProgram();

    const char *vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec3 color;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        uniform float pointSize;
        uniform vec3 tintColor;

        out vec3 vertexColor;

        void main()
        {
            gl_Position = projection * view * model * vec4(position, 1.0);
            gl_PointSize = pointSize;
            vertexColor = color * (tintColor / 255.0);
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        in vec3 vertexColor;
        out vec4 fragColor;

        uniform bool smoothPoints;

        void main()
        {
            if (smoothPoints) {
                vec2 coord = gl_PointCoord - vec2(0.5);
                float dist = length(coord);

                if (dist > 0.5)
                    discard;

                float alpha = 1.0 - smoothstep(0.45, 0.5, dist);
                fragColor = vec4(vertexColor, alpha);
            } else {
                fragColor = vec4(vertexColor, 1.0);
            }
        }
    )";

    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_program->link();
}

void PointCloudGLWidget::loadMesh(const QVector<QVector3D>& vertices, const QVector<unsigned int>& indices)
{
    m_meshVertices = vertices;
    m_meshIndices = indices;
    m_hasMesh = true;
    update();
}

void PointCloudGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_pointClouds.isEmpty())
        return;

    m_view.setToIdentity();
    m_view.translate(0.0f, 0.0f, -m_distance);
    m_view.rotate(m_xRot, 1.0f, 0.0f, 0.0f);
    m_view.rotate(m_yRot, 0.0f, 1.0f, 0.0f);

    if (m_renderMode == POINTS_SMOOTH) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    for (auto it = m_pointClouds.constBegin(); it != m_pointClouds.constEnd(); ++it) {
        const QString& name = it.key();
        const PointCloud& pc = it.value();

        if (pc.isVisible && !pc.points.isEmpty()) {
            renderPointCloud(pc);
        }
    }

    if (m_showBoundingBox && !m_selectedEntityForBoundingBox.isEmpty() && m_pointClouds.contains(m_selectedEntityForBoundingBox)) {
        const PointCloud& pc = m_pointClouds[m_selectedEntityForBoundingBox];
        glLineWidth(2.0f);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMin.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMin.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMax.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMax.y(), pc.boundingBoxMin.z());
        glEnd();
        glBegin(GL_LINE_LOOP);
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMin.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMin.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMax.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMax.y(), pc.boundingBoxMax.z());
        glEnd();
        glBegin(GL_LINES);
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMin.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMin.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMin.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMin.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMax.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMax.x(), pc.boundingBoxMax.y(), pc.boundingBoxMax.z());
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMax.y(), pc.boundingBoxMin.z());
        glVertex3f(pc.boundingBoxMin.x(), pc.boundingBoxMax.y(), pc.boundingBoxMax.z());
        glEnd();
    }

    if (m_renderMode == POINTS_SMOOTH) {
        glDisable(GL_BLEND);
    }

    if (m_hasMesh && !m_meshVertices.isEmpty() && !m_meshIndices.isEmpty()) {
        glColor3f(0.7f, 0.7f, 0.9f);
        glBegin(GL_TRIANGLES);
        for (int i = 0; i + 2 < m_meshIndices.size(); i += 3) {
            const QVector3D& v1 = m_meshVertices[m_meshIndices[i]];
            const QVector3D& v2 = m_meshVertices[m_meshIndices[i + 1]];
            const QVector3D& v3 = m_meshVertices[m_meshIndices[i + 2]];
            glVertex3f(v1.x(), v1.y(), v1.z());
            glVertex3f(v2.x(), v2.y(), v2.z());
            glVertex3f(v3.x(), v3.y(), v3.z());
        }
        glEnd();
    }
}

void PointCloudGLWidget::renderPointCloud(const PointCloud& pc)
{
    if (pc.points.isEmpty())
        return;

    m_program->bind();
    m_program->setUniformValue("model", m_model);
    m_program->setUniformValue("view", m_view);
    m_program->setUniformValue("projection", m_projection);
    m_program->setUniformValue("pointSize", pc.pointSize);
    m_program->setUniformValue("smoothPoints", m_renderMode == POINTS_SMOOTH);

    QVector3D tintColor(pc.tintColor.red(), pc.tintColor.green(), pc.tintColor.blue());
    m_program->setUniformValue("tintColor", tintColor);

    m_vao.bind();
    m_vbo.bind();

    QVector<GLfloat> vertexData;
    vertexData.reserve(pc.points.size() * 6);
    for (int i = 0; i < pc.points.size(); ++i)
    {
        vertexData.append(pc.points[i].x());
        vertexData.append(pc.points[i].y());
        vertexData.append(pc.points[i].z());
        vertexData.append(pc.colors[i].x() / 255.0f);
        vertexData.append(pc.colors[i].y() / 255.0f);
        vertexData.append(pc.colors[i].z() / 255.0f);
    }
    m_vbo.allocate(vertexData.constData(), vertexData.size() * sizeof(GLfloat));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void*>(3 * sizeof(GLfloat)));

    glDrawArrays(GL_POINTS, 0, pc.points.size());

    m_vbo.release();
    m_vao.release();
    m_program->release();

    if (!pc.polygons.isEmpty()) {
        m_program->bind();
        m_program->setUniformValue("model", m_model);
        m_program->setUniformValue("view", m_view);
        m_program->setUniformValue("projection", m_projection);
        m_program->setUniformValue("pointSize", 1.0f);
        m_program->setUniformValue("smoothPoints", false);
        m_program->setUniformValue("tintColor", tintColor);

        QOpenGLVertexArrayObject triangleVAO;
        triangleVAO.create();
        triangleVAO.bind();

        QOpenGLBuffer triangleVBO(QOpenGLBuffer::VertexBuffer);
        triangleVBO.create();
        triangleVBO.bind();

        QVector<GLfloat> triangleData;
        int totalVertices = 0;
        for (const auto& polygon : pc.polygons) {
            totalVertices += polygon.size();
        }
        triangleData.reserve(totalVertices * 6);

        for (int i = 0; i < pc.polygons.size(); ++i) {
            const QVector<QVector3D>& triangle = pc.polygons[i];
            const QVector<QVector3D>& triangleColor = pc.polygonColors[i];

            for (int j = 0; j < triangle.size(); ++j) {
                triangleData.append(triangle[j].x());
                triangleData.append(triangle[j].y());
                triangleData.append(triangle[j].z());
                QVector3D color = triangleColor[j];
                triangleData.append(color.x() / 255.0f);
                triangleData.append(color.y() / 255.0f);
                triangleData.append(color.z() / 255.0f);
            }
        }

        triangleVBO.allocate(triangleData.constData(), triangleData.size() * sizeof(GLfloat));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void*>(3 * sizeof(GLfloat)));

        glDrawArrays(GL_TRIANGLES, 0, totalVertices);

        triangleVBO.release();
        triangleVAO.release();
        m_program->release();
    }
}

void PointCloudGLWidget::resizeGL(int width, int height)
{
    float aspect = width / static_cast<float>(height);
    m_projection.setToIdentity();
    m_projection.perspective(m_fov, aspect, 0.01f, 1000.0f);
    update();
}

void PointCloudGLWidget::setPointClouds(const QMap<QString, PointCloud>& pointClouds)
{
    m_pointClouds = pointClouds;
    update();
}

void PointCloudGLWidget::updatePointCloudVisibility(const QString& name, bool visible)
{
    if (m_pointClouds.contains(name)) {
        m_pointClouds[name].isVisible = visible;
        update();
    }
}

void PointCloudGLWidget::calculateSceneExtents(QVector3D& min, QVector3D& max)
{
    min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    bool foundVisiblePoints = false;

    for (const auto& pc : m_pointClouds) {
        if (pc.isVisible && !pc.points.isEmpty()) {
            for (const auto& point : pc.points) {
                min.setX(qMin(min.x(), point.x()));
                min.setY(qMin(min.y(), point.y()));
                min.setZ(qMin(min.z(), point.z()));

                max.setX(qMax(max.x(), point.x()));
                max.setY(qMax(max.y(), point.y()));
                max.setZ(qMax(max.z(), point.z()));
            }
            foundVisiblePoints = true;
        }
    }

    if (!foundVisiblePoints) {
        min = QVector3D(-1.0f, -1.0f, -1.0f);
        max = QVector3D(1.0f, 1.0f, 1.0f);
    }
}

void PointCloudGLWidget::resetView()
{
    m_xRot = 30.0f; // Increased from 15.0f
    m_yRot = 40.0f; // Increased from 15.0f
    m_distance = 5.0f;
    m_fov = 30.0f; // Changed from 45.0f
    m_focalDistance = 0.5f;

    QVector3D min, max;
    calculateSceneExtents(min, max);

    QVector3D center = (min + max) * 0.5f;
    QVector3D size = max - min;

    // Increase the distance multiplier from 0.8f to 2.0f
    float sceneSize = size.length();
    m_distance = sceneSize * 2.0f;

    // Decrease scale value to show more of the scene - change from 1.5f to 1.0f
    float scale = 1.0f / qMax(qMax(size.x(), size.y()), size.z());

    m_model.setToIdentity();
    m_model.scale(scale);
    m_model.translate(-center);

    update();
}


void PointCloudGLWidget::setFocusOnPointCloud(const QString& name, const QVector3D& min, const QVector3D& max)
{
    if (!m_pointClouds.contains(name))
        return;

    const PointCloud &pc = m_pointClouds[name];
    if (pc.points.isEmpty())
        return;

    // Calculate bounding box center and size
    QVector3D center = (min + max) * 0.5f;
    QVector3D size = max - min;
    float sceneSize = size.length();

    // Increase the distance multiplier from 0.8f to 2.0f for a more zoomed-out view
    float cameraDistance = sceneSize * 4.0f;
    m_distance = cameraDistance;

    m_view.setToIdentity();
    m_view.lookAt(
        center + QVector3D(0.0f, 0.0f, cameraDistance), // Camera position
        center,                                        // Look-at point
        QVector3D(0.0f, 1.0f, 0.0f)                    // Up vector
        );

    // Adjust FOV for better perspective - change from 45.0f to 40.0f for less distortion
    m_fov = 30.0f;
    m_focalDistance = sceneSize * 1.5f;

    float aspect = width() / static_cast<float>(height());
    float nearPlane = qMax(0.01f, sceneSize * 0.01f);
    float farPlane = sceneSize * 20.0f; // Increased from 10.0f to 20.0f for better depth visibility

    m_projection.setToIdentity();
    m_projection.perspective(m_fov, aspect, nearPlane, farPlane);

    // Decrease scale value to show more of the scene - change from 1.5f to 1.0f
    float scale = 1.0f / qMax(qMax(size.x(), size.y()), size.z());
    m_model.setToIdentity();
    m_model.scale(scale);
    m_model.translate(-center);

    // Adjust initial rotation angles for better 3D perspective
    m_xRot = 30.0f; // Increased from 15.0f
    m_yRot = 40.0f; // Increased from 15.0f

    m_selectedEntityForBoundingBox = name;
    m_showBoundingBox = true;
    update();
}

void PointCloudGLWidget::showBoundingBox(const QVector3D& minCorner, const QVector3D& maxCorner)
{
    m_selectedEntityForBoundingBox = "";
    m_showBoundingBox = true;
    update();
}

void PointCloudGLWidget::hideBoundingBox()
{
    m_showBoundingBox = false;
    update();
}

void PointCloudGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void PointCloudGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->position().x() - m_lastPos.x();
    int dy = event->position().y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton)
    {
        m_yRot += dx;
        m_xRot += dy;
        update();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        m_distance -= dy * 0.01f;
        m_distance = qMax(0.1f, m_distance);
        update();
    }

    m_lastPos = event->position().toPoint();
}

void PointCloudGLWidget::wheelEvent(QWheelEvent *event)
{
    m_distance -= event->angleDelta().y() * 0.001f;
    m_distance = qMax(0.1f, m_distance);
    update();
}

// ========== MainWindow Implementation ==========

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setupUI();
    createMenus();

    statusBar()->showMessage(tr("Ready"));
    setWindowTitle(tr("Point Cloud Viewer"));
}

MainWindow::~MainWindow()
{
    qDeleteAll(m_viewportList);
    delete ui;
}

QString MainWindow::getSelectedPointCloud() const
{
    QTreeWidgetItem *currentItem = m_treeWidget->currentItem();
    if (!currentItem)
        return QString();

    QString name = currentItem->data(0, Qt::UserRole).toString();
    if (m_pointClouds.contains(name))
        return name;

    return QString();
}

QString MainWindow::getSupportedFormatsFilter() const
{
    QString filter = tr("Point Cloud Files (*.pts)");

#ifdef USE_ASSIMP
    filter += tr(";;3D Models (*.obj *.fbx *.dae *.3ds *.ply *.stl *.gltf *.glb)");
    filter += tr(";;Wavefront OBJ (*.obj)");
    filter += tr(";;Autodesk FBX (*.fbx)");
    filter += tr(";;COLLADA (*.dae)");
    filter += tr(";;3D Studio Max (*.3ds)");
    filter += tr(";;Stanford PLY (*.ply)");
    filter += tr(";;Stereolithography STL (*.stl)");
    filter += tr(";;GL Transmission Format (*.gltf *.glb)");
    filter += tr(";;All Supported Files (*.pts *.obj *.fbx *.dae *.3ds *.ply *.stl *.gltf *.glb)");
#endif

    filter += tr(";;All Files (*)");
    return filter;
}

void MainWindow::setupUI()
{
    m_glWidget = new PointCloudGLWidget(this);

    QWidget *centralWidget = ui->centralwidget;
    delete centralWidget->layout();
    QHBoxLayout *layout = new QHBoxLayout(centralWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_glWidget);

    QDockWidget *treeDock = ui->dockWidget_3;
    QWidget *treeWidget = treeDock->widget();
    delete treeWidget->layout();

    m_treeWidget = new QTreeWidget(treeWidget);
    m_treeWidget->setHeaderLabel(tr("Files"));
    m_treeWidget->setColumnWidth(0, 200);
    m_treeWidget->setAlternatingRowColors(true);

    m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_treeWidget->setColumnCount(2);
    m_treeWidget->setHeaderLabels(QStringList() << tr("File") << tr("Points"));

    QHBoxLayout *treeLayout = new QHBoxLayout(treeWidget);
    treeLayout->setContentsMargins(0, 0, 0, 0);
    treeLayout->addWidget(m_treeWidget);

    QDockWidget *textDock = ui->dockWidget_2;
    QWidget *textWidget = textDock->widget();
    delete textWidget->layout();

    m_textEdit = new QPlainTextEdit(textWidget);
    m_textEdit->setReadOnly(true);
    QVBoxLayout *textLayout = new QVBoxLayout(textWidget);
    textLayout->setContentsMargins(0, 0, 0, 0);
    textLayout->addWidget(m_textEdit);

    connect(m_treeWidget, &QTreeWidget::itemClicked, this, &MainWindow::onItemClicked);
    connect(m_treeWidget, &QTreeWidget::itemChanged, this, &MainWindow::onItemChanged);
    connect(m_treeWidget, &QTreeWidget::itemDoubleClicked, this, &MainWindow::onItemDoubleClicked);
}

void MainWindow::createMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));

    QAction *openAction = new QAction(tr("&Open"), this);
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &MainWindow::openFile);
    fileMenu->addAction(openAction);

    QAction *exportAction = new QAction(tr("&Export Selected as PTS"), this);
    exportAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
    connect(exportAction, &QAction::triggered, this, &MainWindow::exportPointCloud);
    fileMenu->addAction(exportAction);

    fileMenu->addSeparator();

    QAction *exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    fileMenu->addAction(exitAction);

    QMenu *viewMenu = menuBar()->addMenu(tr("&View"));

    QAction *resetViewAction = new QAction(tr("&Reset View"), this);
    resetViewAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    connect(resetViewAction, &QAction::triggered, this, &MainWindow::resetView);
    viewMenu->addAction(resetViewAction);

    viewMenu->addSeparator();

    QAction *showAllAction = new QAction(tr("Show &All"), this);
    connect(showAllAction, &QAction::triggered, [this]() {
        setAllVisible(true);
    });
    viewMenu->addAction(showAllAction);

    QAction *hideAllAction = new QAction(tr("&Hide All"), this);
    connect(hideAllAction, &QAction::triggered, [this]() {
        setAllVisible(false);
    });
    viewMenu->addAction(hideAllAction);

    viewMenu->addSeparator();

    QMenu *pointSizeMenu = viewMenu->addMenu(tr("Point &Size"));

    QAction *smallPointsAction = new QAction(tr("&Small (2px)"), this);
    connect(smallPointsAction, &QAction::triggered, [this]() {
        m_glWidget->setPointSize(2.0f);
    });
    pointSizeMenu->addAction(smallPointsAction);

    QAction *mediumPointsAction = new QAction(tr("&Medium (4px)"), this);
    connect(mediumPointsAction, &QAction::triggered, [this]() {
        m_glWidget->setPointSize(4.0f);
    });
    pointSizeMenu->addAction(mediumPointsAction);

    QAction *largePointsAction = new QAction(tr("&Large (6px)"), this);
    connect(largePointsAction, &QAction::triggered, [this]() {
        m_glWidget->setPointSize(6.0f);
    });
    pointSizeMenu->addAction(largePointsAction);

    viewMenu->addSeparator();

    QMenu *renderModeMenu = viewMenu->addMenu(tr("Render &Mode"));

    QAction *standardPointsAction = new QAction(tr("&Standard Points"), this);
    connect(standardPointsAction, &QAction::triggered, [this]() {
        m_glWidget->setRenderMode(PointCloudGLWidget::POINTS);
    });
    renderModeMenu->addAction(standardPointsAction);

    QAction *smoothPointsAction = new QAction(tr("S&mooth Points"), this);
    connect(smoothPointsAction, &QAction::triggered, [this]() {
        m_glWidget->setRenderMode(PointCloudGLWidget::POINTS_SMOOTH);
    });
    renderModeMenu->addAction(smoothPointsAction);

    QMenu *viewportMenu = menuBar()->addMenu(tr("Viewport Select/Unselect"));
    QAction *saveViewportAction = new QAction(tr("Save Viewport for Selected Entity"), this);
    connect(saveViewportAction, &QAction::triggered, this, &MainWindow::saveViewportForSelectedEntity);
    viewportMenu->addAction(saveViewportAction);

    QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));

    QAction *aboutAction = new QAction(tr("&About"), this);
    connect(aboutAction, &QAction::triggered, this, &MainWindow::showAbout);
    helpMenu->addAction(aboutAction);
}

void MainWindow::openFile()
{
    QStringList filenames = QFileDialog::getOpenFileNames(
        this, tr("Open 3D Model Files"), QString(), getSupportedFormatsFilter()
        );

    for (const QString &filename : filenames)
    {
        QFileInfo fileInfo(filename);
        QString extension = fileInfo.suffix().toLower();

        bool success = false;

        if (extension == "pts")
        {
            success = loadPointCloud(filename);
        }
#ifdef USE_ASSIMP
        else if (extension == "obj" || extension == "fbx" || extension == "dae" ||
                 extension == "3ds" || extension == "ply" || extension == "stl" ||
                 extension == "gltf" || extension == "glb")
        {
            success = loadModelWithAssimp(filename);
        }
#endif
        else
        {
            success = loadPointCloud(filename);
#ifdef USE_ASSIMP
            if (!success)
            {
                success = loadModelWithAssimp(filename);
            }
#endif
        }

        if (!success)
        {
            QMessageBox::warning(this, tr("Error"),
                                 tr("Failed to load file: %1. Unsupported format or file is corrupted.").arg(filename));
        }
    }

    updateAllVisiblePointClouds();
}

void MainWindow::setAllVisible(bool visible)
{
    m_treeWidget->blockSignals(true);

    for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = m_treeWidget->topLevelItem(i);
        item->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);

        QString name = item->data(0, Qt::UserRole).toString();
        if (m_pointClouds.contains(name)) {
            m_pointClouds[name].isVisible = visible;
        }
    }

    m_treeWidget->blockSignals(false);
    updateAllVisiblePointClouds();
}

void MainWindow::exportPointCloud()
{
    QTreeWidgetItem *currentItem = m_treeWidget->currentItem();
    if (!currentItem)
    {
        QMessageBox::warning(this, tr("Error"), tr("No point cloud selected for export."));
        return;
    }

    QString name = currentItem->data(0, Qt::UserRole).toString();
    if (!m_pointClouds.contains(name))
    {
        QMessageBox::warning(this, tr("Error"), tr("Selected item is not a valid point cloud."));
        return;
    }

    const PointCloud &pc = m_pointClouds[name];
    QString defaultName = name;
    if (!defaultName.endsWith(".pts"))
    {
        defaultName = defaultName.split('.').first() + ".pts";
    }

    QString filename = QFileDialog::getSaveFileName(
        this, tr("Export Point Cloud"), defaultName, tr("Point Cloud Files (*.pts);;All Files (*)")
        );

    if (filename.isEmpty())
        return;

    if (saveAsPts(filename, pc))
    {
        statusBar()->showMessage(tr("Exported %1 points to %2").arg(pc.points.size()).arg(filename));
    }
    else
    {
        QMessageBox::warning(this, tr("Error"), tr("Failed to export point cloud to %1").arg(filename));
    }
}

bool MainWindow::saveAsPts(const QString &filename, const PointCloud &pc)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return false;
    }

    QTextStream out(&file);
    QProgressDialog progress(tr("Exporting point cloud..."), tr("Cancel"), 0, pc.points.size(), this);
    progress.setWindowModality(Qt::WindowModal);

    for (int i = 0; i < pc.points.size(); ++i)
    {
        if (i % 1000 == 0)
        {
            progress.setValue(i);
            if (progress.wasCanceled())
            {
                file.close();
                return false;
            }
        }

        const QVector3D &point = pc.points[i];
        const QVector3D &color = pc.colors[i];

        out << QString("%1 %2 %3 %4 %5 %6\n")
                   .arg(point.x(), 0, 'f', 6)
                   .arg(point.y(), 0, 'f', 6)
                   .arg(point.z(), 0, 'f', 6)
                   .arg(qRound(color.x()))
                   .arg(qRound(color.y()))
                   .arg(qRound(color.z()));
    }

    progress.setValue(pc.points.size());
    file.close();
    return true;
}

bool MainWindow::loadPointCloud(const QString &filename)
{
    try
    {
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::warning(this, tr("Error"), tr("Failed to open file: %1").arg(filename));
            return false;
        }

        PointCloud pc;
        QTextStream in(&file);
        QProgressDialog progress(tr("Loading point cloud..."), tr("Cancel"), 0, 100, this);
        progress.setWindowModality(Qt::WindowModal);

        qint64 fileSize = file.size();
        qint64 bytesRead = 0;
        int lastProgress = 0;

        int lineNumber = 0;
        while (!in.atEnd())
        {
            QString line = in.readLine().trimmed();
            lineNumber++;

            bytesRead += line.length() + 1;
            int currentProgress = static_cast<int>((bytesRead * 100) / fileSize);
            if (currentProgress != lastProgress)
            {
                progress.setValue(currentProgress);
                lastProgress = currentProgress;

                if (progress.wasCanceled())
                {
                    file.close();
                    return false;
                }
            }

            if (line.isEmpty())
                continue;

            QStringList values = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);

            if (values.size() >= 3)
            {
                bool ok1 = false, ok2 = false, ok3 = false;
                float x = values[0].toFloat(&ok1);
                float y = values[1].toFloat(&ok2);
                float z = values[2].toFloat(&ok3);

                if (!ok1 || !ok2 || !ok3)
                {
                    qDebug() << "Error parsing point coordinates at line" << lineNumber;
                    continue;
                }

                pc.points.append(QVector3D(x, y, z));

                if (values.size() >= 6)
                {
                    bool ok4 = false, ok5 = false, ok6 = false;
                    int r = values[3].toInt(&ok4);
                    int g = values[4].toInt(&ok5);
                    int b = values[5].toInt(&ok6);

                    if (!ok4 || !ok5 || !ok6)
                    {
                        qDebug() << "Error parsing color values at line" << lineNumber;
                        pc.colors.append(QVector3D(255, 255, 255));
                    }
                    else
                    {
                        r = qBound(0, r, 255);
                        g = qBound(0, g, 255);
                        b = qBound(0, b, 255);
                        pc.colors.append(QVector3D(r, g, b));
                    }
                }
                else
                {
                    pc.colors.append(QVector3D(255, 255, 255));
                }
            }
        }

        file.close();
        progress.setValue(100);

        if (pc.points.isEmpty())
        {
            QMessageBox::warning(this, tr("Error"), tr("No valid points found in file: %1").arg(filename));
            return false;
        }

        pc.boundingBoxMin = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        pc.boundingBoxMax = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
        for (const auto& point : pc.points) {
            pc.boundingBoxMin.setX(qMin(pc.boundingBoxMin.x(), point.x()));
            pc.boundingBoxMin.setY(qMin(pc.boundingBoxMin.y(), point.y()));
            pc.boundingBoxMin.setZ(qMin(pc.boundingBoxMin.z(), point.z()));
            pc.boundingBoxMax.setX(qMax(pc.boundingBoxMax.x(), point.x()));
            pc.boundingBoxMax.setY(qMax(pc.boundingBoxMax.y(), point.y()));
            pc.boundingBoxMax.setZ(qMax(pc.boundingBoxMax.z(), point.z()));
        }

        pc.sourceFormat = "PTS";
        pc.isVisible = true; // Default to visible

        QColor colors[] = {
            QColor(255, 255, 255),
            QColor(230, 230, 255),
            QColor(230, 255, 230),
            QColor(255, 230, 230),
            QColor(255, 255, 230),
            QColor(230, 255, 255),
            QColor(255, 230, 255)
        };
        pc.tintColor = colors[m_pointClouds.size() % 7];

        QFileInfo fileInfo(filename);
        QString name = fileInfo.fileName();
        m_pointClouds[name] = pc;

        QTreeWidgetItem *item = new QTreeWidgetItem();
        item->setText(0, name);
        item->setData(0, Qt::UserRole, name);
        item->setToolTip(0, filename);
        item->setText(1, QString::number(pc.points.size()));
        item->setCheckState(0, Qt::Checked); // Default to checked
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setIcon(0, QIcon(":/icons/text-x-generic.png"));
        m_treeWidget->addTopLevelItem(item);
        m_treeWidget->expandAll();

        m_treeWidget->setCurrentItem(item);
        statusBar()->showMessage(tr("Loaded %1 with %2 points").arg(name).arg(pc.points.size()));
        displayPointCloudInfo(name, pc);

        // Focus on the newly loaded point cloud
        focusCameraOnPointCloud(name);

        return true;
    }
    catch (const std::exception &e)
    {
        QMessageBox::critical(this, tr("Error"), tr("Error loading file: %1").arg(e.what()));
        return false;
    }
}

void MainWindow::updateAllVisiblePointClouds()
{
    m_glWidget->setPointClouds(m_pointClouds);
}

void MainWindow::onItemChanged(QTreeWidgetItem *item, int column)
{
    if (column == 0)
    {
        QString name = item->data(0, Qt::UserRole).toString();
        if (m_pointClouds.contains(name))
        {
            bool isVisible = (item->checkState(0) == Qt::Checked);
            m_pointClouds[name].isVisible = isVisible;
            m_glWidget->updatePointCloudVisibility(name, isVisible);
            updateAllVisiblePointClouds();
        }
    }
}

void MainWindow::displayPointCloudInfo(const QString &name, const PointCloud &pc)
{
    m_textEdit->clear();
    m_textEdit->appendPlainText(tr("File: %1").arg(name));
    m_textEdit->appendPlainText(tr("Number of points: %1").arg(pc.points.size()));
    m_textEdit->appendPlainText(tr("Format: %1").arg(pc.sourceFormat));
    m_textEdit->appendPlainText(tr("Visible: %1").arg(pc.isVisible ? tr("Yes") : tr("No")));

    if (!pc.points.isEmpty())
    {
        m_textEdit->appendPlainText(QString());
        m_textEdit->appendPlainText(tr("Bounding Box:"));
        m_textEdit->appendPlainText(tr("X: %1 to %2").arg(pc.boundingBoxMin.x()).arg(pc.boundingBoxMax.x()));
        m_textEdit->appendPlainText(tr("Y: %1 to %2").arg(pc.boundingBoxMin.y()).arg(pc.boundingBoxMax.y()));
        m_textEdit->appendPlainText(tr("Z: %1 to %2").arg(pc.boundingBoxMin.z()).arg(pc.boundingBoxMax.z()));

        QVector3D sum(0.0f, 0.0f, 0.0f);
        for (const auto& point : pc.points)
            sum += point;

        QVector3D centroid = sum / pc.points.size();

        m_textEdit->appendPlainText(QString());
        m_textEdit->appendPlainText(tr("Centroid:"));
        m_textEdit->appendPlainText(tr("X: %1").arg(centroid.x()));
        m_textEdit->appendPlainText(tr("Y: %1").arg(centroid.y()));
        m_textEdit->appendPlainText(tr("Z: %1").arg(centroid.z()));
    }
}

void MainWindow::onItemClicked(QTreeWidgetItem *item, int column)
{
    if (!item)
        return;

    QString name = item->data(0, Qt::UserRole).toString();
    if (m_pointClouds.contains(name))
    {
        const PointCloud &pc = m_pointClouds[name];
        displayPointCloudInfo(name, pc);
        focusCameraOnPointCloud(name);
    }
}

void MainWindow::onItemDoubleClicked(QTreeWidgetItem *item, int column)
{
    if (!item)
        return;

    QVariant data = item->data(0, Qt::UserRole);
    if (data.canConvert<ViewportObject*>()) {
        ViewportObject* viewport = data.value<ViewportObject*>();
        viewport->applyViewport(m_glWidget);
        statusBar()->showMessage(tr("Applied viewport: %1").arg(viewport->getName()));
    }
    else {
        QString name = item->data(0, Qt::UserRole).toString();
        if (m_pointClouds.contains(name))
        {
            const PointCloud &pc = m_pointClouds[name];
            displayPointCloudInfo(name, pc);
            focusCameraOnPointCloud(name);
        }
    }
}

void MainWindow::focusCameraOnPointCloud(const QString &name)
{
    if (!m_pointClouds.contains(name))
        return;

    const PointCloud &pc = m_pointClouds[name];
    m_glWidget->setFocusOnPointCloud(name, pc.boundingBoxMin, pc.boundingBoxMax);
    statusBar()->showMessage(tr("Focused on %1").arg(name));
}

void MainWindow::resetView()
{
    m_glWidget->resetView();
    updateAllVisiblePointClouds();
}

void MainWindow::showPointCloudProperties()
{
    QTreeWidgetItem *currentItem = m_treeWidget->currentItem();
    if (!currentItem)
        return;

    QString name = currentItem->data(0, Qt::UserRole).toString();
    if (!m_pointClouds.contains(name))
        return;

    QDialog dialog(this);
    dialog.setWindowTitle(tr("Point Cloud Properties"));

    QVBoxLayout *layout = new QVBoxLayout(&dialog);

    QCheckBox *visibleCheckBox = new QCheckBox(tr("Visible"), &dialog);
    visibleCheckBox->setChecked(m_pointClouds[name].isVisible);
    layout->addWidget(visibleCheckBox);

    QLabel *pointSizeLabel = new QLabel(tr("Point Size:"), &dialog);
    layout->addWidget(pointSizeLabel);

    QSlider *pointSizeSlider = new QSlider(Qt::Horizontal, &dialog);
    pointSizeSlider->setRange(1, 10);
    pointSizeSlider->setValue(m_pointClouds[name].pointSize);
    layout->addWidget(pointSizeSlider);

    QPushButton *colorButton = new QPushButton(tr("Tint Color..."), &dialog);
    layout->addWidget(colorButton);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    connect(colorButton, &QPushButton::clicked, [&]() {
        QColor color = QColorDialog::getColor(m_pointClouds[name].tintColor, this);
        if (color.isValid()) {
            m_pointClouds[name].tintColor = color;
        }
    });

    if (dialog.exec() == QDialog::Accepted) {
        m_pointClouds[name].isVisible = visibleCheckBox->isChecked();
        m_pointClouds[name].pointSize = pointSizeSlider->value();
        currentItem->setCheckState(0, m_pointClouds[name].isVisible ? Qt::Checked : Qt::Unchecked);
        m_glWidget->updatePointCloudVisibility(name, m_pointClouds[name].isVisible);
    }
}

void MainWindow::showAbout()
{
    QMessageBox::about(this, tr("About Point Cloud Viewer"),
                       tr("Point Cloud Viewer\n\n"
                          "A simple application for viewing multiple point cloud and 3D model files simultaneously.\n"
                          "Supports PTS point clouds and various 3D model formats via Assimp."));
}

void MainWindow::saveViewportForSelectedEntity()
{
    QString name = getSelectedPointCloud();
    if (name.isEmpty())
    {
        QMessageBox::warning(this, tr("Error"), tr("No point cloud selected!"));
        return;
    }

    const PointCloud &pc = m_pointClouds[name];
    if (pc.points.isEmpty())
    {
        QMessageBox::warning(this, tr("Error"), tr("Selected point cloud has no valid points."));
        return;
    }

    QVector3D bboxMin = pc.boundingBoxMin;
    QVector3D bboxMax = pc.boundingBoxMax;
    if (bboxMin == bboxMax)
    {
        QMessageBox::warning(this, tr("Error"), tr("Invalid bounding box for selected point cloud!"));
        return;
    }

    QVector3D bboxCenter = (bboxMin + bboxMax) * 1.5f;
    QVector3D bboxSize = bboxMax - bboxMin;
    float sceneSize = bboxSize.length();

    // Use the same improved values as in setFocusOnPointCloud
    float cameraDistance = sceneSize * 2.0f; // Increased from 0.8f
    float focalDistance = sceneSize * 1.5f;
    float fov = 30.0f; // Changed from 45.0f

    m_glWidget->setCameraDistance(cameraDistance);
    m_glWidget->setFocalDistance(focalDistance);
    m_glWidget->setFOV(fov);

    QMatrix4x4 viewMatrix;
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(
        bboxCenter + QVector3D(0.0f, 0.0f, cameraDistance),
        bboxCenter,
        QVector3D(0.0f, 1.0f, 0.0f)
        );
    m_glWidget->setViewMatrix(viewMatrix);

    // Decrease scale value to show more of the scene - change from 1.5f to 1.0f
    float scale = 1.0f / qMax(qMax(bboxSize.x(), bboxSize.y()), bboxSize.z());
    QMatrix4x4 modelMatrix;
    modelMatrix.setToIdentity();
    modelMatrix.scale(scale);
    modelMatrix.translate(-bboxCenter);
    m_glWidget->setModelMatrix(modelMatrix);

    m_glWidget->setXRotation(40.0f); // Increased from 15.0f
    m_glWidget->setYRotation(40.0f); // Increased from 15.0f

    m_glWidget->update();

    ViewportObject* viewportObject = new ViewportObject(QString("Viewport #%1 - %2").arg(++s_viewportIndex).arg(name));
    ViewportObject::ViewportParameters params;
    params.modelMatrix = m_glWidget->getModelMatrix();
    params.viewMatrix = m_glWidget->getViewMatrix();
    params.cameraDistance = m_glWidget->getCameraDistance();
    params.xRot = m_glWidget->getXRotation();
    params.yRot = m_glWidget->getYRotation();
    params.modelCenter = bboxCenter;
    params.focalDistance = m_glWidget->getFocalDistance();
    params.fov = m_glWidget->getFOV();
    viewportObject->setParameters(params);

    m_viewportList.append(viewportObject);
    addViewportToDB(viewportObject, name);

    statusBar()->showMessage(tr("Viewport saved for %1").arg(name));
}

void MainWindow::addViewportToDB(ViewportObject* viewport, const QString& entityName)
{
    updateTreeWidget(viewport, entityName);
}

void MainWindow::updateTreeWidget(ViewportObject* viewport, const QString& entityName)
{
    QTreeWidgetItem* parentItem = nullptr;
    for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = m_treeWidget->topLevelItem(i);
        if (item->data(0, Qt::UserRole).toString() == entityName) {
            parentItem = item;
            break;
        }
    }

    if (!parentItem)
        return;

    QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
    item->setText(0, viewport->getName());
    item->setData(0, Qt::UserRole, QVariant::fromValue(viewport));
    item->setIcon(0, QIcon(":/icons/viewport.png"));
    m_treeWidget->expandAll();
}

#ifdef USE_ASSIMP
bool MainWindow::loadModelWithAssimp(const QString &filename)
{
    try
    {
        Assimp::Importer importer;
        unsigned int flags = aiProcess_Triangulate |
                             aiProcess_JoinIdenticalVertices |
                             aiProcess_SortByPType |
                             aiProcess_GenNormals;

        QProgressDialog progress(tr("Loading 3D model..."), tr("Cancel"), 0, 100, this);
        progress.setWindowModality(Qt::WindowModal);
        progress.setValue(10);

        const aiScene* scene = importer.ReadFile(filename.toStdString(), flags);
        progress.setValue(50);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            QMessageBox::warning(this, tr("Assimp Error"),
                                 tr("Error loading model: %1").arg(importer.GetErrorString()));
            return false;
        }

        PointCloud pc;
        unsigned int totalVertices = 0;
        for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
            totalVertices += scene->mMeshes[i]->mNumVertices;
        }

        progress.setMaximum(totalVertices > 0 ? totalVertices : 100);
        progress.setValue(0);
        progress.setLabelText(tr("Converting model to point cloud..."));

        unsigned int processedVertices = 0;
        for (unsigned int i = 0; i < scene->mNumMeshes; i++)
        {
            const aiMesh* mesh = scene->mMeshes[i];
            aiColor4D diffuse(0.8f, 0.8f, 0.8f, 1.0f);
            if (mesh->mMaterialIndex < scene->mNumMaterials) {
                const aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
                material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
            }

            for (unsigned int j = 0; j < mesh->mNumVertices; j++)
            {
                if (j % 1000 == 0) {
                    progress.setValue(processedVertices + j);
                    if (progress.wasCanceled()) {
                        return false;
                    }
                }

                const aiVector3D& pos = mesh->mVertices[j];
                pc.points.append(QVector3D(pos.x, pos.y, pos.z));

                if (mesh->HasVertexColors(0))
                {
                    const aiColor4D& color = mesh->mColors[0][j];
                    pc.colors.append(QVector3D(color.r * 255, color.g * 255, color.b * 255));
                }
                else
                {
                    pc.colors.append(QVector3D(diffuse.r * 255, diffuse.g * 255, diffuse.b * 255));
                }
            }

            processedVertices += mesh->mNumVertices;
        }

        progress.setValue(totalVertices);

        if (pc.points.isEmpty())
        {
            QMessageBox::warning(this, tr("Error"),
                                 tr("No vertices found in model: %1").arg(filename));
            return false;
        }

        pc.boundingBoxMin = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        pc.boundingBoxMax = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
        for (const auto& point : pc.points) {
            pc.boundingBoxMin.setX(qMin(pc.boundingBoxMin.x(), point.x()));
            pc.boundingBoxMin.setY(qMin(pc.boundingBoxMin.y(), point.y()));
            pc.boundingBoxMin.setZ(qMin(pc.boundingBoxMin.z(), point.z()));
            pc.boundingBoxMax.setX(qMax(pc.boundingBoxMax.x(), point.x()));
            pc.boundingBoxMax.setY(qMax(pc.boundingBoxMax.y(), point.y()));
            pc.boundingBoxMax.setZ(qMax(pc.boundingBoxMax.z(), point.z()));
        }

        QFileInfo fileInfo(filename);
        pc.sourceFormat = fileInfo.suffix().toUpper();
        pc.isVisible = true; // Default to visible

        QColor colors[] = {
            QColor(255, 255, 255),
            QColor(230, 230, 255),
            QColor(230, 255, 230),
            QColor(255, 230, 230),
            QColor(255, 255, 230),
            QColor(230, 255, 255),
            QColor(255, 230, 255)
        };
        pc.tintColor = colors[m_pointClouds.size() % 7];

        QString name = fileInfo.fileName();
        m_pointClouds[name] = pc;

        QTreeWidgetItem *item = new QTreeWidgetItem();
        item->setText(0, name);
        item->setData(0, Qt::UserRole, name);
        item->setToolTip(0, filename);
        item->setText(1, QString::number(pc.points.size()));
        item->setCheckState(0, Qt::Checked); // Default to checked
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setIcon(0, QIcon(":/icons/model.png"));
        m_treeWidget->addTopLevelItem(item);
        m_treeWidget->expandAll();

        m_treeWidget->setCurrentItem(item);
        statusBar()->showMessage(tr("Loaded %1 with %2 points").arg(name).arg(pc.points.size()));
        displayPointCloudInfo(name, pc);

        focusCameraOnPointCloud(name);

        return true;
    }
    catch (const std::exception &e)
    {
        QMessageBox::critical(this, tr("Error"),
                              tr("Error loading model: %1").arg(e.what()));
        return false;
    }
}
#endif
