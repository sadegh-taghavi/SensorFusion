#ifndef BASEITEM_H
#define BASEITEM_H

#include <QQuickItem>
#include <QVector3D>
#include <QMatrix4x4>


class SimpleKalmanFilter
{

public:
  SimpleKalmanFilter(float mea_e = 0.0001f, float est_e = 0.0001f, float q = 0.001f );
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;

};



class BaseItem : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QVector3D axis READ axis WRITE setAxis NOTIFY axisChanged)
    Q_PROPERTY(float angle READ angle WRITE setAngle NOTIFY angleChanged)
    QVector3D m_axis;

    float m_angle;
    int m_count;
    QVector3D m_baseReading;
    QVector3D m_avgReading;
    SimpleKalmanFilter m_kf[3];
public:
    explicit BaseItem(QQuickItem *parent = nullptr);

    static void registerTypes();

    Q_INVOKABLE void calculate(float dx, float dy, float dz);

    QVector3D axis() const;

    float angle() const;

signals:

    void axisChanged(QVector3D axis);

    void angleChanged(float angle);

public slots:
void setAxis(QVector3D axis);
void setAngle(float angle);
};

#endif // BASEITEM_H
