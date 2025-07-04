#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>
#include <cmath>
#include <string>

/**
 * @brief Diese Node steuert den Roboter nach einem definierten Bewegungsmuster (z. B. Kreis, Welle).
 * 
 * Ziel ist es, verschiedene Bewegungsprofile zu simulieren, um z. B. die Leistung von Lokalisierungsalgorithmen
 * unter verschiedenen Trajektorien zu evaluieren.
 */
class PatternDriver : public rclcpp::Node {
public:
    PatternDriver()
    : Node("pattern_driver"), phase_(0.0) {
        // Publisher für Geschwindigkeitsbefehle (mit Zeitstempel)
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        // Parameter deklarieren mit Standardwerten
        this->declare_parameter("pattern", "circle");        // Bewegungsmuster
        this->declare_parameter("linear_speed", 0.2);        // Vorwärtsgeschwindigkeit [m/s]
        this->declare_parameter("angular_speed", 0.5);       // Rotationsgeschwindigkeit [rad/s]
        this->declare_parameter("duration", 20.0);           // Gesamtdauer der Bewegung [s]

        // Startzeit für Ablaufsteuerung
        start_time_ = this->now();

        // Timer zur regelmäßigen Ausführung der update()-Funktion
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PatternDriver::update, this));

        RCLCPP_INFO(this->get_logger(), "PatternDriver gestartet (Muster: circle | zigzag | wave | etc.)");
    }

private:
    /**
     * @brief Hauptlogik zur Mustersteuerung. Wird periodisch aufgerufen.
     * 
     * Berechnet die aktuelle Bewegung entsprechend dem gewählten Muster und publiziert
     * ein `TwistStamped`-Nachricht zur Steuerung des Roboters.
     */
    void update() {
        auto now = this->now();
        double elapsed = (now - start_time_).seconds();

        // Parameter abrufen
        std::string pattern = this->get_parameter("pattern").as_string();
        double v = this->get_parameter("linear_speed").as_double();
        double w = this->get_parameter("angular_speed").as_double();
        double duration = this->get_parameter("duration").as_double();

        // Initialisiere Befehl
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now;
        cmd.header.frame_id = "base_link";

        // Stoppe Roboter nach Ablauf der Zeit
        if (elapsed > duration) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            publisher_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Bewegung abgeschlossen.");
            rclcpp::shutdown();
            return;
        }

        // === Unterstützte Bewegungsmuster ===
        if (pattern == "circle") {
            // Gleichförmige Kreisbewegung
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = w;
        }
        else if (pattern == "wave") {
            // Sinusförmige Drehung für Wellenbewegung
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = std::sin(phase_) * w * 2.0;
            phase_ += 0.2;
        }
        else if (pattern == "circle_straight_circle") {
            // Abfolge: Kreis → Geradeaus → Kreis
            if (elapsed < 10.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = w;
            } else if (elapsed < 13.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = 0.0;
            } else if (elapsed < 18.0) {
                cmd.twist.linear.x = v;
                cmd.twist.angular.z = w;
            } else {
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Muster abgeschlossen.");
                rclcpp::shutdown();
                return;
            }
        }
        else {
            // Fallback: Kreisbewegung bei unbekanntem Muster
            RCLCPP_WARN_ONCE(this->get_logger(), "Unbekanntes Pattern '%s'. Fallback auf Kreis.", pattern.c_str());
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = w;
        }

        // Befehl veröffentlichen
        publisher_->publish(cmd);
    }

    // === Membervariablen ===
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    double phase_;  ///< Phase zur Erzeugung von sinusförmigen Mustern
};

// === Einstiegspunkt der Node ===
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatternDriver>());
    rclcpp::shutdown();
    return 0;
}
