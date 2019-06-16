#ifndef BASEITEM_H
#define BASEITEM_H

#include <QQuickItem>
#include <QVector3D>
#include <QMatrix4x4>


class KalmanFilter
{

public:
  KalmanFilter(float measurementError = 0.0002f, float estimationError = 0.0002f, float rate = 1.0f );
  float updateEstimate(float measurement);
  void setMeasurementError(float measurementError);
  void setEstimationError(float estimationError);
  void setProcessNoise(float rate);
  float getKalmanGain();
  float getEstimationError();

private:
  float m_measurementError;
  float m_estimationError;
  float m_rate;
  float m_currentEstimation;
  float m_lastEstimation;
  float m_kalmanGain;
};



class BaseItem : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QVector3D axis READ axis WRITE setAxis NOTIFY axisChanged)
    Q_PROPERTY(float angle READ angle WRITE setAngle NOTIFY angleChanged)
    QVector3D m_axis;
    float m_angle;
    KalmanFilter m_kf[3];
    QQuaternion m_baseRotation;
    QQuaternion m_currentRotation;
public:
    explicit BaseItem(QQuickItem *parent = nullptr);

    static void registerTypes();

    Q_INVOKABLE void calculate(float dx, float dy, float dz);
    Q_INVOKABLE void resetRotation();

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
