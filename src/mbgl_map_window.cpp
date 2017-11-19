#include <mbgl_map_window.h>

#include <QApplication>
#include <QColor>
#include <QDebug>
#include <QFile>
#include <QIcon>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>

#if QT_VERSION >= 0x050000
#include <QWindow>
#endif

#include <ros/console.h>
#include <rosrm/RouteService.h>
#include <rosrm/MatchService.h>
#include <rosrm/Overview.h>

int kAnimationDuration = 10000;

const QString layer3Dbuildings = QString::fromLatin1("3d-buildings");

MapboxGLMapWindow::MapboxGLMapWindow(const QMapboxGLSettings &settings)
    : m_map(nullptr)
    , m_mapboxGLSettings(settings)
    , m_currentStyleIndex(0)
{
    m_styles = QMapbox::defaultStyles();
    const auto user_style = qgetenv("MAPBOX_STYLE_URL");
    if (!user_style.isEmpty())
    {
        m_styles.insert(0, {user_style, "User style"});
    }

    const auto user_styles = qgetenv("MAPBOX_STYLE_URLS");
    if (!user_styles.isEmpty())
    {
        for (const auto &style : user_styles.split(';'))
        {
            const auto &name_url = style.split('=');
            m_styles.insert(0, {name_url.size() == 2 ? name_url[1] : name_url[0], name_url.size() == 2 ? name_url[0] : "User style"});
        }
    }
}

void MapboxGLMapWindow::flyTo(double latitude, double longitude, double bearing)
{
    std::cout << "fly to " << latitude << " " <<longitude << " " << bearing << " " << m_mapboxGLCameraOptions.pitch.toDouble() << " " << m_mapboxGLCameraOptions.zoom.toDouble() << " " << m_mapboxGLCameraOptions.anchor.toPointF().x() << "," << m_mapboxGLCameraOptions.anchor.toPointF().y() <<  "\n";
    m_mapboxGLCameraOptions.center = QVariant::fromValue(QMapbox::Coordinate(latitude, longitude));
    m_mapboxGLCameraOptions.angle = bearing;
    m_mapboxGLCameraOptions.zoom = m_map->zoom();
    m_mapboxGLCameraOptions.pitch = m_map->pitch();
    m_map->jumpTo(m_mapboxGLCameraOptions);

    return ;
    if (m_latitudeAnimation) {
        m_latitudeAnimation->setDuration(kAnimationDuration);
        m_latitudeAnimation->setEndValue(latitude);
        m_latitudeAnimation->start();
    }
    if (m_longitudeAnimation) {
        m_longitudeAnimation->setDuration(kAnimationDuration);
        m_longitudeAnimation->setEndValue(longitude);
        m_longitudeAnimation->start();
    }
    if (m_bearingAnimation) {
        auto diff = std::fmod(bearing - m_map->bearing(), 360.);
        if (diff > 180) diff -= 360.;

        std::cout << m_map->bearing() << " -> " << bearing << " diff " << diff << "\n";
        m_bearingAnimation->setDuration(kAnimationDuration);
        m_bearingAnimation->setEndValue(m_map->bearing() + diff);
        m_bearingAnimation->start();
    }
}

qreal MapboxGLMapWindow::pixelRatio() {
#if QT_VERSION >= 0x050600
    return devicePixelRatioF();
#elif QT_VERSION >= 0x050000
    return devicePixelRatio();
#else
    return 1;
#endif
}

void MapboxGLMapWindow::animationFinished()
{
    ROS_INFO("Animation ticks/s: %f", m_animationTicks / static_cast<float>(kAnimationDuration) * 1000.);
    ROS_INFO("Frame draws/s: %f", m_frameDraws / static_cast<float>(kAnimationDuration) * 1000.);
}

void MapboxGLMapWindow::animationValueChanged(const QVariant& value)
{
    std::cout << "value = " << value.toDouble() << "\n";
    m_animationTicks++;
}

void MapboxGLMapWindow::resetNorth()
{
    m_map->setBearing(0);
    m_map->setPitch(0);
}

void MapboxGLMapWindow::setStyle(int style)
{
    m_currentStyleIndex = style % m_styles.size() + (style < 0 ? m_styles.size() : 0);
    m_map->setStyleUrl(m_styles[m_currentStyleIndex].first);
    setWindowTitle(QString("Mapbox GLX: ") + m_styles[m_currentStyleIndex].second);
}

void MapboxGLMapWindow::toggleBuildingsExtrusion()
{
    if (!m_map->layerExists(layer3Dbuildings))
    {
        QVariantMap buildings;
        buildings["id"] = layer3Dbuildings;
        buildings["source"] = "composite";
        buildings["source-layer"] = "building";
        buildings["type"] = "fill-extrusion";
        buildings["minzoom"] = 15.0;
        m_map->addLayer(buildings);

        QVariantList buildingsFilterExpression;
        buildingsFilterExpression.append("==");
        buildingsFilterExpression.append("extrude");
        buildingsFilterExpression.append("true");

        QVariantList buildingsFilter;
        buildingsFilter.append(buildingsFilterExpression);

        m_map->setFilter(layer3Dbuildings, buildingsFilterExpression);

        m_map->setPaintProperty(layer3Dbuildings, "fill-extrusion-color", "#aaa");
        m_map->setPaintProperty(layer3Dbuildings, "fill-extrusion-opacity", .6);

        QVariantMap extrusionHeight;
        extrusionHeight["type"] = "identity";
        extrusionHeight["property"] = "height";

        m_map->setPaintProperty(layer3Dbuildings, "fill-extrusion-height", extrusionHeight);

        QVariantMap extrusionBase;
        extrusionBase["type"] = "identity";
        extrusionBase["property"] = "min_height";

        m_map->setPaintProperty(layer3Dbuildings, "fill-extrusion-base", extrusionBase);

        m_3dbuildings = true;
    }
    else
    {
        m_3dbuildings = !m_3dbuildings;
        m_map->setLayoutProperty(layer3Dbuildings, "visibility", m_3dbuildings ? "visible" : "none");
    }
}

void MapboxGLMapWindow::placeCar(const QMapbox::Coordinate &position, double bearing)
{
    QMapbox::CoordinatesCollections point { { { position } } };
    QMapbox::Feature carPoint { QMapbox::Feature::PointType, point, {}, {} };

    QVariantMap carPosition;
    carPosition["data"] = QVariant::fromValue<QMapbox::Feature>(carPoint);
    if (!m_map->sourceExists("carPosition"))
    {
        carPosition["type"] = "geojson";
        m_map->addSource("carPosition", carPosition);
    }
    else
    {
        m_map->updateSource("carPosition", carPosition);
    }

    if (!m_map->layerExists("carSymbol"))
    {
        m_map->addImage("car-icon", QImage(":car-icon.svg"));

        QVariantMap carSymbol;
        carSymbol["id"] = "carSymbol";
        carSymbol["type"] = "symbol";
        carSymbol["source"] = "carPosition";
        m_map->addLayer(carSymbol);

        m_map->setLayoutProperty("carSymbol", "icon-image", "car-icon");
        m_map->setLayoutProperty("carSymbol", "icon-rotation-alignment", "map");
    }

    m_map->setLayoutProperty("carSymbol", "icon-rotate", bearing);
    m_map->setLayoutProperty("carSymbol", "icon-size", std::pow(2., m_map->zoom() - 18.));
}

void MapboxGLMapWindow::toggleCar()
{
    if (m_map->layerExists("carSymbol"))
    {
        m_carVisible = !m_carVisible;
        std::cout << "m_carVisible " << m_carVisible << "\n";
        m_map->setLayoutProperty("carSymbol", "visibility", m_carVisible ? "visible" : "none");
    }
}

void MapboxGLMapWindow::initializeGL()
{
    ROS_INFO("initializeGL");

    if (!m_map)
    {
        std::lock_guard<std::mutex> lock(m_settings_lock);
        m_map = new QMapboxGL(nullptr, m_mapboxGLSettings, size(), pixelRatio());
        connect(m_map, SIGNAL(needsRendering()), this, SLOT(update()));

        m_latitudeAnimation.reset(new QPropertyAnimation(m_map, "latitude"));
        m_longitudeAnimation.reset(new QPropertyAnimation(m_map, "longitude"));
        m_bearingAnimation.reset(new QPropertyAnimation(m_map, "bearing"));
        connect(m_bearingAnimation.data(), SIGNAL(finished()), this, SLOT(animationFinished()));
        connect(m_bearingAnimation.data(), SIGNAL(valueChanged(const QVariant&)), this, SLOT(animationValueChanged(const QVariant&)));

        connect(m_map, SIGNAL(mapChanged(QMapboxGL::MapChange)), this, SLOT(onMapChanged(QMapboxGL::MapChange)));
    }
}

void MapboxGLMapWindow::paintGL()
{
    m_frameDraws++;
#if QT_VERSION >= 0x050400
    // When we're using QOpenGLWidget, we need to tell Mapbox GL about the framebuffer we're using.
    m_map->setFramebufferObject(defaultFramebufferObject());
#endif
    m_map->render();
}

void MapboxGLMapWindow::resizeGL(int w, int h)
{
    QSize size(w, h);
    m_map->resize(size, size * pixelRatio());
}

void MapboxGLMapWindow::saveSettings(qt_gui_cpp::Settings &settings) const
{
    std::lock_guard<std::mutex> lock(m_settings_lock);

    settings.setValue("styleIndex", QVariant::fromValue(m_currentStyleIndex));

    if (m_map)
    {
        settings.setValue("latitude", m_map->latitude());
        settings.setValue("longitude", m_map->longitude());
        settings.setValue("zoom", m_map->zoom());
        settings.setValue("bearing", m_map->bearing());
        settings.setValue("pitch", m_map->pitch());
    }
    else
    {
        settings.setValue("latitude", m_mapboxGLCameraOptions.center.value<QMapbox::Coordinate>().first);
        settings.setValue("longitude", m_mapboxGLCameraOptions.center.value<QMapbox::Coordinate>().second);
        settings.setValue("zoom", m_mapboxGLCameraOptions.zoom);
        settings.setValue("bearing", m_mapboxGLCameraOptions.angle);
        settings.setValue("pitch", m_mapboxGLCameraOptions.pitch);
    }
}

void MapboxGLMapWindow::restoreSettings(const qt_gui_cpp::Settings &settings)
{
    std::lock_guard<std::mutex> lock(m_settings_lock);

    m_currentStyleIndex = settings.value("styleIndex", 0).toUInt();

    m_mapboxGLCameraOptions = {
        QVariant::fromValue(QMapbox::Coordinate(settings.value("latitude", 48.13727).toDouble(),
                                                settings.value("longitude", 11.575506).toDouble())),
        {0, 0},
        settings.value("zoom", 14).toDouble(),
        settings.value("bearing", 0).toDouble(),
        settings.value("pitch", 0).toDouble()
    };

    // Update the map
    m_map->jumpTo(m_mapboxGLCameraOptions);
    setStyle(m_currentStyleIndex);
}

bool MapboxGLMapWindow::event(QEvent *e)
{
    switch (e->type()) {
    case QEvent::WindowChangeInternal:
        if (m_map)
        {
            std::lock_guard<std::mutex> lock(m_settings_lock);
            m_mapboxGLCameraOptions = {
                QVariant::fromValue(m_map->coordinate()),
                {0, 0},
                m_map->zoom(),
                m_map->bearing(),
                m_map->pitch()};
            m_map = nullptr;
        };
        break;
    }

    return QOpenGLWidget::event(e);
}

void MapboxGLMapWindow::onMapChanged(QMapboxGL::MapChange change)
{
    switch (change)
    {
    case QMapboxGL::MapChangeRegionWillChange:
        //std::cout << "zoom " << m_map->zoom() << "\n";
        //placeCar(m_currentCarPosition, m_currentCarBearing);
        break;
    default:
        //std::cout << "MapboxGLMapWindow::mapChanged " << change << "\n";
        break;
    }
}

void MapboxGLMapWindow::keyPressEvent(QKeyEvent *ev)
{
    // ROS_INFO("keyPressEvent %d", ev->key());

    switch (ev->key())
    {
        /*
    case Qt::Key_Up:
        m_currentCarPosition.first += 0.001;
        placeCar(m_currentCarPosition, 0);
        break;
    case Qt::Key_Down:
        m_currentCarPosition.first -= 0.001;
        placeCar(m_currentCarPosition, 0);
        break;
    case Qt::Key_Right:
        m_currentCarPosition.second += 0.001;
        placeCar(m_currentCarPosition, 0);
        break;
    case Qt::Key_Left:
        m_currentCarPosition.second -= 0.001;
        placeCar(m_currentCarPosition, 0);
        break;
        */

    case Qt::Key_R:
        m_routeStart = {48.295211667, 11.894606667};
        m_routeDestination = {48.177228333, 11.589738333};
        findRoute(m_routeStart, m_routeDestination);
        break;

    case Qt::Key_T:
        flyTo(40,40,0);
        break;

    case Qt::Key_X:
        resetNorth();
        break;

    case Qt::Key_S:
        setStyle(m_currentStyleIndex + ((ev->modifiers() & Qt::ShiftModifier) ? -1 : 1));
        break;

    case Qt::Key_E:
        toggleBuildingsExtrusion();
        break;

    case Qt::Key_Z:

        if (!m_map->sourceExists("circleSource"))
        {
            QMapbox::CoordinatesCollections point { { { {48.13727, 11.575506} } } };
            QMapbox::Feature feature { QMapbox::Feature::PointType, point, {}, {} };

            QVariantMap circleSource;
            circleSource["type"] = "geojson";
            circleSource["data"] = QVariant::fromValue<QMapbox::Feature>(feature);
            m_map->addSource("circleSource", circleSource);

            QVariantMap circle;
            circle["id"] = "circleLayer";
            circle["type"] = "circle";
            circle["source"] = "circleSource";
            m_map->addLayer(circle);

            m_map->setPaintProperty("circleLayer", "circle-radius", 10.0);
            m_map->setPaintProperty("circleLayer", "circle-color", QColor("black"));
        }
        else
        {
            QMapbox::CoordinatesCollections point { { { {48.13727, 11.577506} } } };
            QMapbox::Feature feature { QMapbox::Feature::PointType, point, {}, {} };

            QVariantMap circleSource;
            circleSource["data"] = QVariant::fromValue<QMapbox::Feature>(feature);
            m_map->updateSource("circleSource", circleSource);

        }
        break;

    case Qt::Key_C:
        toggleCar();
        break;

    case Qt::Key_L: {
            if (m_sourceAdded) {
                return;
            }

            m_sourceAdded = true;

            // Not in all styles, but will work on streets
            QString before = "waterway-label";

            QFile geojson(":source1.geojson");
            geojson.open(QIODevice::ReadOnly);

            // The data source for the route line and markers
            QVariantMap routeSource;
            routeSource["type"] = "geojson";
            routeSource["data"] = geojson.readAll();
            std::cout << geojson.readAll().constData() << "\n";
            m_map->addSource("routeSource", routeSource);

            // The route case, painted before the route
            QVariantMap routeCase;
            routeCase["id"] = "routeCase";
            routeCase["type"] = "line";
            routeCase["source"] = "routeSource";
            m_map->addLayer(routeCase, before);

            m_map->setPaintProperty("routeCase", "line-color", QColor("white"));
            m_map->setPaintProperty("routeCase", "line-width", 20.0);
            m_map->setLayoutProperty("routeCase", "line-join", "round");
            m_map->setLayoutProperty("routeCase", "line-cap", "round");

            // The route, painted on top of the route case
            QVariantMap route;
            route["id"] = "route";
            route["type"] = "line";
            route["source"] = "routeSource";
            m_map->addLayer(route, before);

            m_map->setPaintProperty("route", "line-color", QColor("blue"));
            m_map->setPaintProperty("route", "line-width", 8.0);
            m_map->setLayoutProperty("route", "line-join", "round");
            m_map->setLayoutProperty("route", "line-cap", "round");

            QVariantList lineDashArray;
            lineDashArray.append(1);
            lineDashArray.append(2);

            m_map->setPaintProperty("route", "line-dasharray", lineDashArray);

            // Markers at the beginning and end of the route
            m_map->addImage("label-arrow", QImage(":label-arrow.svg"));
            m_map->addImage("label-background", QImage(":label-background.svg"));

            QVariantMap markerArrow;
            markerArrow["id"] = "markerArrow";
            markerArrow["type"] = "symbol";
            markerArrow["source"] = "routeSource";
            m_map->addLayer(markerArrow);

            m_map->setLayoutProperty("markerArrow", "icon-image", "label-arrow");
            m_map->setLayoutProperty("markerArrow", "icon-size", 0.5);
            m_map->setLayoutProperty("markerArrow", "icon-ignore-placement", true);

            QVariantList arrowOffset;
            arrowOffset.append(0.0);
            arrowOffset.append(-15.0);
            m_map->setLayoutProperty("markerArrow", "icon-offset", arrowOffset);

            QVariantMap markerBackground;
            markerBackground["id"] = "markerBackground";
            markerBackground["type"] = "symbol";
            markerBackground["source"] = "routeSource";
            m_map->addLayer(markerBackground);

            m_map->setLayoutProperty("markerBackground", "icon-image", "label-background");
            m_map->setLayoutProperty("markerBackground", "text-field", "{name}");
            m_map->setLayoutProperty("markerBackground", "icon-text-fit", "both");
            m_map->setLayoutProperty("markerBackground", "icon-ignore-placement", true);
            m_map->setLayoutProperty("markerBackground", "text-ignore-placement", true);
            m_map->setLayoutProperty("markerBackground", "text-anchor", "left");
            m_map->setLayoutProperty("markerBackground", "text-size", 16.0);
            m_map->setLayoutProperty("markerBackground", "text-padding", 0.0);
            m_map->setLayoutProperty("markerBackground", "text-line-height", 1.0);
            m_map->setLayoutProperty("markerBackground", "text-max-width", 8.0);

            QVariantList iconTextFitPadding;
            iconTextFitPadding.append(15.0);
            iconTextFitPadding.append(10.0);
            iconTextFitPadding.append(15.0);
            iconTextFitPadding.append(10.0);
            m_map->setLayoutProperty("markerBackground", "icon-text-fit-padding", iconTextFitPadding);

            QVariantList backgroundOffset;
            backgroundOffset.append(-0.5);
            backgroundOffset.append(-1.5);
            m_map->setLayoutProperty("markerBackground", "text-offset", backgroundOffset);

            m_map->setPaintProperty("markerBackground", "text-color", QColor("white"));

            QVariantList filterExpression;
            filterExpression.append("==");
            filterExpression.append("$type");
            filterExpression.append("Point");

            QVariantList filter;
            filter.append(filterExpression);

            m_map->setFilter("markerArrow", filter);
            m_map->setFilter("markerBackground", filter);

            // Tilt the labels when tilting the map and make them larger
            m_map->setLayoutProperty("road-label-large", "text-size", 30.0);
            m_map->setLayoutProperty("road-label-large", "text-pitch-alignment", "viewport");

            m_map->setLayoutProperty("road-label-medium", "text-size", 30.0);
            m_map->setLayoutProperty("road-label-medium", "text-pitch-alignment", "viewport");

            m_map->setLayoutProperty("road-label-small", "text-pitch-alignment", "viewport");
            m_map->setLayoutProperty("road-label-small", "text-size", 30.0);

            // Buildings extrusion
            QVariantMap buildings;
            buildings["id"] = "3d-buildings";
            buildings["source"] = "composite";
            buildings["source-layer"] = "building";
            buildings["type"] = "fill-extrusion";
            buildings["minzoom"] = 15.0;
            m_map->addLayer(buildings);

            QVariantList buildingsFilterExpression;
            buildingsFilterExpression.append("==");
            buildingsFilterExpression.append("extrude");
            buildingsFilterExpression.append("true");

            QVariantList buildingsFilter;
            buildingsFilter.append(buildingsFilterExpression);

            m_map->setFilter("3d-buildings", buildingsFilterExpression);

            m_map->setPaintProperty("3d-buildings", "fill-extrusion-color", "#aaa");
            m_map->setPaintProperty("3d-buildings", "fill-extrusion-opacity", .6);

            QVariantMap extrusionHeight;
            extrusionHeight["type"] = "identity";
            extrusionHeight["property"] = "height";

            m_map->setPaintProperty("3d-buildings", "fill-extrusion-height", extrusionHeight);

            QVariantMap extrusionBase;
            extrusionBase["type"] = "identity";
            extrusionBase["property"] = "min_height";

            m_map->setPaintProperty("3d-buildings", "fill-extrusion-base", extrusionBase);
        }
        break;
    case Qt::Key_1: {
            if (m_symbolAnnotationId.isNull()) {
                QMapbox::Coordinate coordinate = m_map->coordinate();
                QMapbox::SymbolAnnotation symbol { coordinate, "default_marker" };
                m_map->addAnnotationIcon("default_marker", QImage(":default_marker.svg"));
                m_symbolAnnotationId = m_map->addAnnotation(QVariant::fromValue<QMapbox::SymbolAnnotation>(symbol));
            } else {
                m_map->removeAnnotation(m_symbolAnnotationId.toUInt());
                m_symbolAnnotationId.clear();
            }
        }
        break;
    case Qt::Key_2: {
            if (m_lineAnnotationId.isNull()) {
                QMapbox::Coordinate topLeft     = m_map->coordinateForPixel({ 0, 0 });
                QMapbox::Coordinate bottomRight = m_map->coordinateForPixel({ qreal(size().width()), qreal(size().height()) });
                QMapbox::CoordinatesCollections lineGeometry { { { topLeft, bottomRight } } };
                QMapbox::ShapeAnnotationGeometry annotationGeometry { QMapbox::ShapeAnnotationGeometry::LineStringType, lineGeometry };
                QMapbox::LineAnnotation line;
                line.geometry = annotationGeometry;
                line.opacity = 0.5f;
                line.width = 1.0f;
                line.color = Qt::red;
                m_lineAnnotationId = m_map->addAnnotation(QVariant::fromValue<QMapbox::LineAnnotation>(line));
            } else {
                m_map->removeAnnotation(m_lineAnnotationId.toUInt());
                m_lineAnnotationId.clear();
            }
        }
        break;
    case Qt::Key_3: {
            if (m_fillAnnotationId.isNull()) {
                QMapbox::Coordinate topLeft     = m_map->coordinateForPixel({ 0, 0 });
                QMapbox::Coordinate topRight    = m_map->coordinateForPixel({ 0, qreal(size().height()) });
                QMapbox::Coordinate bottomLeft  = m_map->coordinateForPixel({ qreal(size().width()), 0 });
                QMapbox::Coordinate bottomRight = m_map->coordinateForPixel({ qreal(size().width()), qreal(size().height()) });
                QMapbox::CoordinatesCollections fillGeometry { { { bottomLeft, bottomRight, topRight, topLeft, bottomLeft } } };
                QMapbox::ShapeAnnotationGeometry annotationGeometry { QMapbox::ShapeAnnotationGeometry::PolygonType, fillGeometry };
                QMapbox::FillAnnotation fill;
                fill.geometry = annotationGeometry;
                fill.opacity = 0.5f;
                fill.color = Qt::green;
                fill.outlineColor = QVariant::fromValue<QColor>(QColor(Qt::black));
                m_fillAnnotationId = m_map->addAnnotation(QVariant::fromValue<QMapbox::FillAnnotation>(fill));
            } else {
                m_map->removeAnnotation(m_fillAnnotationId.toUInt());
                m_fillAnnotationId.clear();
            }
        }
        break;
    case Qt::Key_5: {
            if (m_map->layerExists("circleLayer")) {
                m_map->removeLayer("circleLayer");
                m_map->removeSource("circleSource");
            } else {
                QMapbox::CoordinatesCollections point { { { m_map->coordinate() } } };
                QMapbox::Feature feature { QMapbox::Feature::PointType, point, {}, {} };

                QVariantMap circleSource;
                circleSource["type"] = "geojson";
                circleSource["data"] = QVariant::fromValue<QMapbox::Feature>(feature);
                m_map->addSource("circleSource", circleSource);

                QVariantMap circle;
                circle["id"] = "circleLayer";
                circle["type"] = "circle";
                circle["source"] = "circleSource";
                m_map->addLayer(circle);

                m_map->setPaintProperty("circleLayer", "circle-radius", 10.0);
                m_map->setPaintProperty("circleLayer", "circle-color", QColor("black"));
            }
        }
        break;
    case Qt::Key_D:
        m_map->cycleDebugOptions();
        break;
    default:
        break;
    }

    ev->accept();
}

void MapboxGLMapWindow::mousePressEvent(QMouseEvent *ev)
{
#if QT_VERSION < 0x050000
    m_lastPos = ev->posF();
#else
    m_lastPos = ev->localPos();
#endif

    if (ev->type() == QEvent::MouseButtonPress)
    {
        if (ev->modifiers() & Qt::ControlModifier)
        {
            m_routeStart = m_map->coordinateForPixel(m_lastPos);
            findRoute(m_routeStart, m_routeDestination);
        }
        else if (ev->modifiers() & Qt::MetaModifier)
        {
            m_routeDestination = m_map->coordinateForPixel(m_lastPos);
            findRoute(m_routeStart, m_routeDestination);
        }
    }

    if (ev->type() == QEvent::MouseButtonDblClick)
    {
        if (ev->buttons() == Qt::LeftButton)
        {
            m_map->scaleBy(2.0, m_lastPos);
        }
        else if (ev->buttons() == Qt::RightButton)
        {
            m_map->scaleBy(0.5, m_lastPos);
        }
    }

    ev->accept();
}

void MapboxGLMapWindow::mouseMoveEvent(QMouseEvent *ev)
{
    // ROS_INFO("mouseMoveEvent %g %g", m_map->pitch(), m_map->bearing());

#if QT_VERSION < 0x050000
    QPointF delta = ev->posF() - m_lastPos;
#else
    QPointF delta = ev->localPos() - m_lastPos;
#endif

    if (!delta.isNull()) {
        if (ev->buttons() == Qt::LeftButton && ev->modifiers() & Qt::ShiftModifier) {
            m_map->setPitch(m_map->pitch() - delta.y());
        } else if (ev->buttons() == Qt::LeftButton) {
            m_map->moveBy(delta);
        } else if (ev->buttons() == Qt::RightButton) {
#if QT_VERSION < 0x050000
            m_map->rotateBy(m_lastPos, ev->posF());
#else
            m_map->rotateBy(m_lastPos, ev->localPos());
#endif
        }
    }

#if QT_VERSION < 0x050000
    m_lastPos = ev->posF();
#else
    m_lastPos = ev->localPos();
#endif
    ev->accept();
}

void MapboxGLMapWindow::wheelEvent(QWheelEvent *ev)
{
    if (ev->orientation() == Qt::Horizontal) {
        return;
    }

    float factor = ev->delta() / 1200.;
    if (ev->delta() < 0) {
        factor = factor > -1 ? factor : 1 / factor;
    }

    m_map->scaleBy(1 + factor, ev->pos());
    ev->accept();
}

void MapboxGLMapWindow::findRoute(const QMapbox::Coordinate &from, const QMapbox::Coordinate &to)
{
    ROS_INFO("Find route from %g,%g to %g,%g", from.first, from.second, to.first, to.second);

    ros::ServiceClient router = nh.serviceClient<rosrm::RouteService::Request, rosrm::RouteService::Response>("/rosrm_server/route");

    rosrm::RouteService::Request request;
    rosrm::RouteService::Response response;

    request.waypoints.resize(2);
    request.waypoints[0].position.x = from.second;
    request.waypoints[0].position.y = from.first;
    request.waypoints[1].position.x = to.second;
    request.waypoints[1].position.y = to.first;
    request.overview = rosrm::Overview::Full;
    request.number_of_alternatives = 0;

    router.call(request, response);

    QMapbox::Coordinates routeGeometry;
    for (const auto &coordinate : response.routes[0].coordinates)
    {
        routeGeometry.push_back({coordinate.y, coordinate.x});
    }

    QVariantMap routeSource;
    QMapbox::Feature feature{QMapbox::Feature::LineStringType, QMapbox::CoordinatesCollections{{routeGeometry}}, {}, {}};
    routeSource["data"] = QVariant::fromValue<QMapbox::Feature>(feature);

    if (!m_map->sourceExists("routeSource"))
    {
        routeSource["type"] = "geojson";
        m_map->addSource("routeSource", routeSource);

        QVariantMap routeLayer;
        routeLayer["id"] = "routeLayer";
        routeLayer["type"] = "line";
        routeLayer["source"] = "routeSource";
        m_map->addLayer(routeLayer);

        m_map->setPaintProperty("routeLayer", "line-color", QColor("#4b93d6"));
        m_map->setPaintProperty("routeLayer", "line-width", 20.0);
        m_map->setLayoutProperty("routeLayer", "line-join", "round");
        m_map->setLayoutProperty("routeLayer", "line-cap", "round");
    }
    else
    {
        m_map->updateSource("routeSource", routeSource);
    }
}
