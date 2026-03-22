#include "mks/ScopeWidget.h"
#include <QDateTime>
#include <algorithm>
#include <cmath>

namespace RDT {

ScopeWidget::ScopeWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(200);
}

ScopeWidget::~ScopeWidget() {}

void ScopeWidget::addChannel(const QString& name, const QColor& color) {
    if (!channels_.contains(name)) {
        ScopeChannel ch;
        ch.name = name;
        ch.color = color;
        ch.visible = true;
        channels_.insert(name, ch);
    }
}

void ScopeWidget::addData(const QString& name, double x, double y) {
    if (channels_.contains(name)) {
        auto& ch = channels_[name];
        if (ch.data.capacity() < MAX_POINTS) {
            ch.data.reserve(MAX_POINTS);
        }

        if (ch.data.size() < MAX_POINTS) {
            ch.data.append(QPointF(x, y));
            ch.size = ch.data.size();
            ch.head = 0;
        } else {
            // Ring-buffer overwrite without shifting memory.
            const int write_index = (ch.head + ch.size) % MAX_POINTS;
            if (ch.size < MAX_POINTS) {
                ch.data[write_index] = QPointF(x, y);
                ++ch.size;
            } else {
                ch.data[ch.head] = QPointF(x, y);
                ch.head = (ch.head + 1) % MAX_POINTS;
                ch.size = MAX_POINTS;
            }
        }
    }

    // Coalesce multiple addData calls and cap repaint rate.
    if (!isVisible()) {
        return;
    }

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    const qint64 next_allowed_in_ms = repaint_interval_ms_ - (now_ms - last_repaint_ms_);
    if (next_allowed_in_ms > 0) {
        if (!deferred_repaint_pending_) {
            deferred_repaint_pending_ = true;
            QTimer::singleShot(static_cast<int>(next_allowed_in_ms), this, [this]() {
                deferred_repaint_pending_ = false;
                if (!isVisible()) return;
                last_repaint_ms_ = QDateTime::currentMSecsSinceEpoch();
                update();
            });
        }
        return;
    }

    if (!repaint_scheduled_) {
        repaint_scheduled_ = true;
        QTimer::singleShot(0, this, [this]() {
            repaint_scheduled_ = false;
            last_repaint_ms_ = QDateTime::currentMSecsSinceEpoch();
            update();
        });
    }
}

void ScopeWidget::setChannelVisibility(const QString& name, bool visible) {
    if (channels_.contains(name)) {
        if (channels_[name].visible == visible) {
            return;
        }
        channels_[name].visible = visible;
        update();
    }
}

void ScopeWidget::setYRange(double min, double max) {
    y_min_ = min;
    y_max_ = max;
    update();
}

void ScopeWidget::setAutoRange(bool enabled) {
    auto_range_ = enabled;
    update();
}

void ScopeWidget::setXRange(double range_sec) {
    x_range_ = range_sec;
    update();
}

void ScopeWidget::clear() {
    for (auto& ch : channels_) {
        ch.data.clear();
        ch.head = 0;
        ch.size = 0;
    }
    update();
}

void ScopeWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    // Background
    painter.fillRect(rect(), QColor(30, 30, 30));
    
    double w = width();
    double h = height();
    
    // Determine Time Window (X Axis)
    // We assume X is time in seconds. Use the latest timestamp from any channel to determine the end.
    double max_x = 0.0;
    bool has_data = false;
    
    for (const auto& ch : channels_) {
        if (ch.size > 0 && !ch.data.isEmpty()) {
            const int tail_index = (ch.head + ch.size - 1) % ch.data.size();
            max_x = std::max(max_x, ch.data[tail_index].x());
            has_data = true;
        }
    }
    
    if (!has_data) {
        // Draw grid only
        painter.setPen(QColor(60, 60, 60));
        painter.drawRect(rect().adjusted(0,0,-1,-1));
        return;
    }
    
    double min_x = max_x - x_range_;
    
    // Auto Range Y Calculation
    if (auto_range_) {
        const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
        if (now_ms - last_autorange_update_ms_ >= autorange_update_interval_ms_) {
            double min_val = 1e9;
            double max_val = -1e9;
            bool range_found = false;

            for (const auto& ch : channels_) {
                if (!ch.visible) continue;
                if (ch.size == 0 || ch.data.isEmpty()) continue;

                for (int i = 0; i < ch.size; ++i) {
                    const int idx = (ch.head + i) % ch.data.size();
                    const auto& p = ch.data[idx];
                    if (p.x() >= min_x) {
                        min_val = std::min(min_val, p.y());
                        max_val = std::max(max_val, p.y());
                        range_found = true;
                    }
                }
            }

            if (range_found) {
                double margin = (max_val - min_val) * 0.1;
                if (margin == 0) margin = 1.0;
                y_min_ = min_val - margin;
                y_max_ = max_val + margin;
            }
            last_autorange_update_ms_ = now_ms;
        }
    }
    
    // Grid lines
    painter.setPen(QColor(60, 60, 60));
    // Horizontal lines
    int h_lines = 5;
    for (int i = 0; i <= h_lines; ++i) {
        double y = h * i / h_lines;
        painter.drawLine(0, y, w, y);
    }
    // Vertical lines
    int v_lines = 10;
    for (int i = 0; i <= v_lines; ++i) {
        double x = w * i / v_lines;
        painter.drawLine(x, 0, x, h);
    }
    
    // Map functions
    auto mapX = [&](double x) {
        return (x - min_x) / x_range_ * w;
    };
    auto mapY = [&](double y) {
        if (std::abs(y_max_ - y_min_) < 1e-6) return h/2.0;
        return h - (y - y_min_) / (y_max_ - y_min_) * h;
    };
    
    // Draw Channels (polyline is lighter than QPainterPath for dense updates)
    for (auto it = channels_.begin(); it != channels_.end(); ++it) {
        const auto& ch = it.value();
        if (!ch.visible || ch.size == 0 || ch.data.empty()) continue;

        QPolygonF poly;
        poly.reserve(ch.size);

        for (int i = 0; i < ch.size; ++i) {
            const int idx = (ch.head + i) % ch.data.size();
            const auto& p = ch.data[idx];
            if (p.x() < min_x) continue;

            double px = mapX(p.x());
            double py = mapY(p.y());
            poly.append(QPointF(px, py));
        }

        if (poly.size() < 2) continue;

        painter.setPen(QPen(ch.color, 2));
        painter.drawPolyline(poly);
    }
    
    // Draw Legend / Info
    painter.setPen(Qt::white);
    int y_offset = 20;
    for (auto it = channels_.begin(); it != channels_.end(); ++it) {
        const auto& ch = it.value();
        QString val_str = "---";
        if (ch.size > 0 && !ch.data.isEmpty()) {
            const int tail_index = (ch.head + ch.size - 1) % ch.data.size();
            val_str = QString::number(ch.data[tail_index].y(), 'f', 2);
        }
        
        painter.setPen(ch.color);
        painter.drawText(10, y_offset, QString("%1: %2").arg(ch.name).arg(val_str));
        y_offset += 15;
    }
}

} // namespace RDT
