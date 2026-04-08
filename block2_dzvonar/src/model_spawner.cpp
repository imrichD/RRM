#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ModelSpawner : public rclcpp::Node
{
public:
  ModelSpawner() : Node("model_spawner")
  {
    // Vytvorenie publishera na požadovaný topic
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    
    // Timer, ktorý publikuje marker každú 1 sekundu
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ModelSpawner::publish_marker, this));
  }

private:
  void publish_marker()
  {
    visualization_msgs::msg::Marker marker;
    
    // Fixná poloha voči základni robota
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "work_object";
    marker.id = 0;
    
    // Typ markera: externý 3D model
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Cesta k tvojmu STL súboru (musí sedieť s názvom balíka!)
    marker.mesh_resource = "package://block2_dzvonar/meshes/tie.stl";

    // Umiestnenie objektu v priestore (X, Y, Z). 
    // Zatiaľ ho dáme 1 meter pred robota, neskôr si hodnoty upravíš.
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.5;
    
    // Orientácia (quaternion) - nulová rotácia
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Mierka modelu. Ak je tvoj model exportovaný v milimetroch a je v Rvize 
    // obrovský, zmeň tieto hodnoty na 0.001
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Farba objektu (Sivá, plne nepriehľadná)
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModelSpawner>());
  rclcpp::shutdown();
  return 0;
}