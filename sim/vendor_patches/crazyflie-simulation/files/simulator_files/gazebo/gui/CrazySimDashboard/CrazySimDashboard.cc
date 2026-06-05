#include "CrazySimDashboard.hh"

#include <gz/plugin/Register.hh>

#include <QMetaObject>
#include <QQmlContext>

using namespace crazyflie_interface;

GZ_ADD_PLUGIN(
  CrazySimDashboard,
  gz::gui::Plugin
)

GZ_ADD_PLUGIN_ALIAS(
  CrazySimDashboard,
  "CrazySimDashboard"
)

CrazySimDashboard::CrazySimDashboard()
  : text("Waiting for /crazysim/dashboard ..."),
    topic("/crazysim/dashboard")
{
}

QString CrazySimDashboard::Text() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->text;
}

QString CrazySimDashboard::Topic() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->topic;
}

void CrazySimDashboard::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->Context())
  {
    this->Context()->setContextProperty("Dashboard", this);
  }

  if (_pluginElem)
  {
    if (auto topicElem = _pluginElem->FirstChildElement("topic"))
    {
      if (topicElem->GetText())
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->topic = QString::fromStdString(topicElem->GetText());
        emit this->TopicChanged();
      }
    }
  }

  const auto topicStd = this->Topic().toStdString();
  this->node.Subscribe(topicStd, &CrazySimDashboard::OnDashboardMessage, this);
}

void CrazySimDashboard::SetText(const QString &_text)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->text == _text)
    {
      return;
    }
    this->text = _text;
  }

  emit this->TextChanged();
}

void CrazySimDashboard::OnDashboardMessage(const gz::msgs::StringMsg &_msg)
{
  const QString next = QString::fromStdString(_msg.data());
  QMetaObject::invokeMethod(
    this,
    "SetText",
    Qt::QueuedConnection,
    Q_ARG(QString, next));
}
