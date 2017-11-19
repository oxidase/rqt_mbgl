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

const QString layer3Dbuildings = QString::fromLatin1("3d-buildings");

namespace
{
const constexpr double DEGREE_TO_RAD = M_PI / 180.;
const constexpr double RAD_TO_DEGREE = 180. / M_PI;
const constexpr double EARTH_RADIUS = 6371008.8; // IUGG  mean radius (2a+b)/3

double computeDistance(const QMapbox::Coordinate& from, const QMapbox::Coordinate &to)
{
    // Use haversine formula
    const auto lat1 = from.first * DEGREE_TO_RAD;
    const auto lng1 = from.second * DEGREE_TO_RAD;
    const auto lat2 = to.first * DEGREE_TO_RAD;
    const auto lng2 = to.second * DEGREE_TO_RAD;
    const auto dlng = lng1 - lng2;
    const auto dlat = lat1 - lat2;
    const auto aharv = std::pow(std::sin(dlat / 2.0), 2.0) +
                         std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlng / 2.), 2);
    const auto charv = 2. * std::atan2(std::sqrt(aharv), std::sqrt(1.0 - aharv));
    return EARTH_RADIUS * charv;
}

double computeBearing(const QMapbox::Coordinate& from, const QMapbox::Coordinate &to)
{
    const auto lng_delta = (to.second - from.second) * DEGREE_TO_RAD;
    const auto lat1 = from.first * DEGREE_TO_RAD;
    const auto lat2 = to.first * DEGREE_TO_RAD;
    const auto y = std::sin(lng_delta) * std::cos(lat2);
    const auto x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(lng_delta);
    return std::atan2(y, x) * RAD_TO_DEGREE;
}
}

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

void MapboxGLMapWindow::jumpTo(const QMapbox::Coordinate &to, double bearing)
{
    m_mapboxGLCameraOptions.center = QVariant::fromValue(to);
    m_mapboxGLCameraOptions.angle = bearing;
    m_mapboxGLCameraOptions.zoom = m_map->zoom();
    m_mapboxGLCameraOptions.pitch = m_map->pitch();
    m_map->jumpTo(m_mapboxGLCameraOptions);
}

void MapboxGLMapWindow::flyTo(const QMapbox::Coordinate &to, double bearing, int duration)
{
    if (m_latitudeAnimation)
    {
        m_latitudeAnimation->setDuration(duration);
        m_latitudeAnimation->setEndValue(to.first);
        m_latitudeAnimation->start();
    }
    if (m_longitudeAnimation) {
        m_longitudeAnimation->setDuration(duration);
        m_longitudeAnimation->setEndValue(to.second);
        m_longitudeAnimation->start();
    }
    if (m_bearingAnimation)
    {
         auto diff = std::fmod(bearing - m_map->bearing(), 360.);
         if (diff > +180) diff -= 360.;
         if (diff < -180) diff += 360.;

         m_bearingAnimation->setDuration(duration);
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
    // ROS_INFO("Animation ticks/s: %f", m_animationTicks / static_cast<float>(kAnimationDuration) * 1000.);
    // ROS_INFO("Frame draws/s: %f", m_frameDraws / static_cast<float>(kAnimationDuration) * 1000.);
}

void MapboxGLMapWindow::animationValueChanged(const QVariant& value)
{
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
    setWindowTitle(QString("Mapbox GL: ") + m_styles[m_currentStyleIndex].second);
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
        break;
    default:
        break;
    }
}

void MapboxGLMapWindow::keyPressEvent(QKeyEvent *ev)
{
    switch (ev->key())
    {
    case Qt::Key_R:
        m_routeStart = {48.295211667, 11.894606667};
        m_routeDestination = {48.177228333, 11.589738333};
        findRoute(m_routeStart, m_routeDestination);
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

    ros::ServiceClient router = m_nodeHandle.serviceClient<rosrm::RouteService::Request, rosrm::RouteService::Response>("/rosrm_server/route");

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
        m_map->addLayer(routeLayer, "matchLayer");

        m_map->setPaintProperty("routeLayer", "line-color", QColor("#4b93d6"));
        m_map->setPaintProperty("routeLayer", "line-width", 16.0);
        m_map->setLayoutProperty("routeLayer", "line-join", "round");
        m_map->setLayoutProperty("routeLayer", "line-cap", "round");
    }
    else
    {
        m_map->updateSource("routeSource", routeSource);
    }
}

void MapboxGLMapWindow::showMapMatching(const QMapbox::Coordinates &fixes)
{
    ros::ServiceClient matcher = m_nodeHandle.serviceClient<rosrm::MatchService::Request, rosrm::MatchService::Response>("/rosrm_server/match");

    rosrm::MatchService::Request request;
    rosrm::MatchService::Response response;

    request.waypoints.resize(fixes.size());
    for (std::size_t i = 0; i < fixes.size(); ++i)
    {
        request.waypoints[i].pose.position.x = fixes[i].second;
        request.waypoints[i].pose.position.y = fixes[i].first;
    }
    request.overview = rosrm::Overview::Full;
    request.number_of_alternatives = 0;

    matcher.call(request, response);

    if (response.code != "Ok")
        return;

    QMapbox::Coordinates matchGeometry;
    for (const auto &coordinate : response.matchings[0].coordinates)
    {
        matchGeometry.push_back({coordinate.y, coordinate.x});
    }

    {
        QVariantMap matchSource;
        QMapbox::Feature matchFeature{QMapbox::Feature::LineStringType, QMapbox::CoordinatesCollections{{matchGeometry}}, {}, {}};
        matchSource["data"] = QVariant::fromValue<QMapbox::Feature>(matchFeature);

        if (!m_map->sourceExists("matchSource"))
        {
            matchSource["type"] = "geojson";
            m_map->addSource("matchSource", matchSource);

            QVariantMap matchLayer;
            matchLayer["id"] = "matchLayer";
            matchLayer["type"] = "line";
            matchLayer["source"] = "matchSource";
            m_map->addLayer(matchLayer);

            m_map->setPaintProperty("matchLayer", "line-color", QColor("#FFB6C1"));
            m_map->setPaintProperty("matchLayer", "line-width", 20.0);
            m_map->setLayoutProperty("matchLayer", "line-join", "round");
            m_map->setLayoutProperty("matchLayer", "line-cap", "round");
        }
        else
        {
            m_map->updateSource("matchSource", matchSource);
        }
    }

    { // Place fixes points on the map
        QVariantMap fixesSource;
        QMapbox::Feature fixesFeature{QMapbox::Feature::PointType, QMapbox::CoordinatesCollections{{fixes}}, {}, {}};
        fixesSource["data"] = QVariant::fromValue<QMapbox::Feature>(fixesFeature);

        if (!m_map->sourceExists("fixesSource"))
        {
            fixesSource["type"] = "geojson";
            m_map->addSource("fixesSource", fixesSource);

            QVariantMap fixesLayer;
            fixesLayer["id"] = "fixesLayer";
            fixesLayer["type"] = "circle";
            fixesLayer["source"] = "fixesSource";
            m_map->addLayer(fixesLayer);

            m_map->setPaintProperty("fixesLayer", "circle-radius", 5.);
            m_map->setPaintProperty("fixesLayer", "circle-color", QColor("blue"));
        }
        else
        {
            m_map->updateSource("fixesSource", fixesSource);
        }
    }

    const auto bearing = computeBearing(matchGeometry.front(), matchGeometry.back());

    // { // Place car symbol on the map
    //     QVariantMap carSource;
    //     QMapbox::Feature carFeature{QMapbox::Feature::PointType, QMapbox::CoordinatesCollections{{{fixes.back()}}}, {}, {} };
    //     carSource["data"] = QVariant::fromValue<QMapbox::Feature>(carFeature);

    //     if (!m_map->sourceExists("carSource"))
    //     {
    //         carSource["type"] = "geojson";
    //         m_map->addSource("carSource", carSource);

    //         m_map->addImage("car-icon", QImage(":car-icon.svg"));

    //         QVariantMap carLayer;
    //         carLayer["id"] = "carLayer";
    //         carLayer["type"] = "symbol";
    //         carLayer["source"] = "carSource";
    //         m_map->addLayer(carLayer);

    //         m_map->setLayoutProperty("carLayer", "icon-image", "car-icon");
    //         m_map->setLayoutProperty("carLayer", "icon-rotation-alignment", "map");
    //     }
    //     else
    //     {
    //         m_map->updateSource("carSource", carSource);
    //     }

    //     m_map->setLayoutProperty("carLayer", "icon-rotate", bearing);
    //     m_map->setLayoutProperty("carLayer", "icon-size", std::pow(2., m_map->zoom() - 20.));
    // }

    flyTo(matchGeometry.back(), bearing, 500);
}
