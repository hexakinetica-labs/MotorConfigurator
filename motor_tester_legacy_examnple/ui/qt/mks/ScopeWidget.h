#ifndef SCOPE_WIDGET_H
#define SCOPE_WIDGET_H

#include <QWidget>
#include <QMap>
#include <QVector>
#include <QColor>
#include <QTimer>
#include <QPainter>
#include <QPainterPath>
#include <QDateTime>
#include <QPolygonF>
#include <functional>

namespace RDT {

struct ScopeChannel {
    QString name;
    QColor color;
    QVector<QPointF> data;
    int head = 0; // ring buffer start
    int size = 0; // number of valid points in ring
    bool visible = true;
};

class ScopeWidget : public QWidget {
    Q_OBJECT
public:
    explicit ScopeWidget(QWidget* parent = nullptr);
    ~ScopeWidget();

    void addChannel(const QString& name, const QColor& color);
    void addData(const QString& name, double x, double y);
    void addDataBatch(const QString& name, const QVector<QPointF>& points);
    void setChannelVisibility(const QString& name, bool visible);
    
    void setYRange(double min, double max);
    void setAutoRange(bool enabled);
    void setXRange(double range_sec);
    void clear();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void appendPointToChannel(ScopeChannel& channel, double x, double y);
    void scheduleRepaint();
    void appendDecimatedPolyline(const ScopeChannel& channel,
                                 double min_x,
                                 const std::function<double(double)>& map_x,
                                 const std::function<double(double)>& map_y,
                                 double viewport_width,
                                 QPolygonF& out_polyline) const;

    QMap<QString, ScopeChannel> channels_;
    bool repaint_scheduled_ = false;
    bool deferred_repaint_pending_ = false;
    qint64 last_repaint_ms_ = 0;
    int repaint_interval_ms_ = 16; // ~60 FPS
    qint64 last_autorange_update_ms_ = 0;
    int autorange_update_interval_ms_ = 120;
    
    // Viewport settings
    double y_min_ = -180.0;
    double y_max_ = 180.0;
    double x_range_ = 10.0; // Seconds to show
    bool auto_range_ = false;
    
    // Limits for pruning
    static const int MAX_POINTS = 12000;
};

} // namespace RDT

#endif // SCOPE_WIDGET_H
