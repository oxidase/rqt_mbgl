#ifndef MAP_WIDGET_H
#define MAP_WIDGET_H

#include <QtGlobal>

#include <QOpenGLWidget>
#include <QPropertyAnimation>
#include <QScopedPointer>

#include <QMapboxGL>

#include <qt_gui_cpp/settings.h>

#include <mutex>

class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

class MapboxGLMapWindow : public QOpenGLWidget
{
    Q_OBJECT

public:
    MapboxGLMapWindow(const QMapboxGLSettings &);

    void saveSettings(qt_gui_cpp::Settings &settings) const;
    void restoreSettings(const qt_gui_cpp::Settings &settings);

    void placeCar(const QMapbox::Coordinate &position, double bearing);

public slots:
    void flyTo(double latitude, double longitude, double bearing);

protected slots:
    void animationValueChanged(const QVariant& value);
    void animationFinished();

private:
    qreal pixelRatio();

    void resetNorth();
    void setStyle(std::size_t style);
    void toggleBuildingsExtrusion();
    void toggleCar();

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

    // Here must be QScopedPointer<QMapboxGL> instead of a naked pointer
    // Memory leak here is intentional as Mapbox GL clears global context
    QMapboxGL *m_map;

    mutable std::mutex m_settings_lock;
    QMapboxGLSettings m_mapboxGLSettings;
    QMapboxGLCameraOptions m_mapboxGLCameraOptions;
    std::size_t m_currentStyleIndex;

    bool m_3dbuildings = false;

    bool m_carVisible = false;

    QPointF m_lastPos;

    std::mutex m_flyToMutex;
    QScopedPointer<QPropertyAnimation> m_latitudeAnimation;
    QScopedPointer<QPropertyAnimation> m_longitudeAnimation;
    QScopedPointer<QPropertyAnimation> m_bearingAnimation;

    unsigned m_animationTicks = 0;
    unsigned m_frameDraws = 0;

    QVariant m_symbolAnnotationId;
    QVariant m_lineAnnotationId;
    QVariant m_fillAnnotationId;

    bool m_sourceAdded = false;

    QList<QPair<QString, QString> > styles;

private slots:
    void onMapChanged(QMapboxGL::MapChange change);
};

#endif
