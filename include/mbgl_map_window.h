#ifndef MAP_WIDGET_H
#define MAP_WIDGET_H

#include <QtGlobal>

#include <QOpenGLWidget>
#include <QPropertyAnimation>
#include <QScopedPointer>

#include <QMapboxGL>

#include <mutex>

class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

class MapboxGLMapWindow : public QOpenGLWidget
{
    Q_OBJECT

public:
    MapboxGLMapWindow(const QMapboxGLSettings &);

    void selfTest();

    QMapboxGL& getMap() const;
    std::mutex& getMapLock();

protected slots:
    void animationValueChanged();
    void animationFinished();

private:
    void changeStyle();
    qreal pixelRatio();

    // QWidget implementation.
    void keyPressEvent(QKeyEvent *ev) final;
    void mousePressEvent(QMouseEvent *ev) final;
    void mouseMoveEvent(QMouseEvent *ev) final;
    void wheelEvent(QWheelEvent *ev) final;

    // Q{,Open}GLWidget implementation.
    void initializeGL() final;
    void paintGL() final;
    void resizeGL(int w, int h);
    bool event(QEvent *e);

    QPointF m_lastPos;

    QMapboxGLSettings m_settings;
    QScopedPointer<QMapboxGL> m_map;
    std::mutex map_lock;

    bool m_3dbuildings = false;

    QPropertyAnimation *m_bearingAnimation;
    QPropertyAnimation *m_zoomAnimation;

    unsigned m_animationTicks = 0;
    unsigned m_frameDraws = 0;

    QVariant m_symbolAnnotationId;
    QVariant m_lineAnnotationId;
    QVariant m_fillAnnotationId;

    bool m_sourceAdded = false;

    std::size_t currentStyleIndex = 0;
    QList<QPair<QString, QString> > styles;
};

#endif
