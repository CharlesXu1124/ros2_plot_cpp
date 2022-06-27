#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>

#include <memory>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
// #include <sensor_msgs/msg/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <tabulate/table.hpp>

#include <iostream>

using std::placeholders::_1;

using namespace cv;
using namespace tabulate;
using namespace std;
using Row_t = Table::Row_t;

int counter;
int delay;
Table table, chart;

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const
    {
        return std::chrono::duration_cast<second_>(clock_::now() - beg_).count();
    }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1>> second_;
    std::chrono::time_point<clock_> beg_;
};

Timer tmr;

class Ros2PlotNode : public rclcpp::Node
{
public:
    Ros2PlotNode()
        : Node("img_subscriber")
    {
        subscriber_node = this->create_subscription<sensor_msgs::msg::Image>(
            "/carla/ego_vehicle/rgb_front/image", 10, std::bind(&Ros2PlotNode::topic_callback, this, _1));

        table.add_row({"Payload Type", "DDS", "Single-trip Latency (ms)"});
        table.add_row({"Image", "Cyclone DDS", "0.015"});
        table.add_row({"Image", "Connext RTI", "0.015"});

        // Set width of cells in each column
        table.column(0).format().width(40);
        table.column(1).format().width(30);
        table.column(2).format().width(30);

        // Iterate over cells in the first row
        for (auto &cell : table[0])
        {
            cell.format()
                .font_style({FontStyle::underline})
                .font_align(FontAlign::center);
        }

        // Iterator over cells in the first column
        for (auto &cell : table.column(0))
        {
            if (cell.get_text() != "Company")
            {
                cell.format()
                    .font_align(FontAlign::right);
            }
        }

        // Iterate over rows in the table
        size_t index = 0;
        for (auto &row : table)
        {
            row.format()
                .font_style({FontStyle::bold});

            // Set blue background color for alternate rows
            if (index > 0 && index == 2)
            {
                for (auto &cell : row)
                {
                    cell.format()
                        .font_background_color(Color::blue);
                }
            }
            else if (index > 0 && index == 1)
            {
                for (auto &cell : row)
                {
                    cell.format()
                        .font_background_color(Color::yellow);
                }
            }

            index += 1;
        }

        cout << table << endl;

        cout << "\n\n";

        // add chart to the terminal
        chart.format()
            .font_color(Color::white)
            .padding_left(0)
            .padding_right(0)
            .column_separator("")
            .hide_border();

        for (size_t i = 0; i < 10; ++i)
        {
            Row_t row;
            row.push_back(std::to_string(0.03 - i * 0.001));
            for (size_t j = 0; j <= 50; ++j)
            {
                row.push_back(" ");
            }
            chart.add_row(row);
        }

        Row_t row;

        chart.add_row(row);
        chart.add_row(Row_t{});

        chart.column(0).format().padding_left(1).padding_right(1).border_left(" ");

        for (size_t i = 1; i <= 18; ++i)
        {
            chart.column(i).format().width(2);
        }

        chart.column(2).format().border_color(Color::white).border_left("|").border_top("-");

        chart.column(2)[10].format().background_color(Color::yellow);
        chart.column(2)[9].format().background_color(Color::yellow);
        chart.column(2)[8].format().background_color(Color::yellow);
        chart.column(2)[7].format().background_color(Color::yellow);
        chart.column(2)[6].format().background_color(Color::yellow);
        chart.column(2)[5].format().background_color(Color::yellow);
        chart.column(2)[4].format().background_color(Color::yellow);
        chart.column(2)[3].format().background_color(Color::yellow);
        chart.column(2)[2].format().background_color(Color::yellow);
        chart.column(2)[1].format().background_color(Color::yellow);

        chart.column(4)[10].format().background_color(Color::blue);
        chart.column(4)[9].format().background_color(Color::blue);
        chart.column(4)[8].format().background_color(Color::blue);
        chart.column(4)[7].format().background_color(Color::blue);
        chart.column(4)[6].format().background_color(Color::blue);

        chart[2][15].format().background_color(Color::yellow);
        chart[2][16].set_text("Cyclone DDS");
        chart.column(16).format().padding_left(1).width(20);

        chart[4][15].format().background_color(Color::blue);
        chart[4][16].set_text("Connext RTI");

        // move the cursor position to top left of the terminal
        printf("\033[%d;%dH", 1, 24);

        // clear the screen after initiating
        system("clear");

        cout << table;

        cout << "\n\n";

        cout << chart;

        counter = 0;

        tmr.reset();
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr frame) const
    {
        ++counter;
        double t = tmr.elapsed();

        size_t index = 0;
        for (auto &row : table)
        {
            if (index > 0 && index == 1)
            {
                size_t col_num = 0;
                for (auto &cell : row)
                {
                    if (col_num == 2)
                        cell.set_text(to_string(t / 5.0));
                    ++col_num;
                }
            }
            ++index;
        }

        // update the chart
        for (int i = 10; i > 0; --i)
        {
            if (int((t / 5.0 - 0.02) * 1000) > (10 - i))
                chart.column(2)[i].format().background_color(Color::yellow);
            else
                chart.column(2)[i].format().background_color(Color::none);
        }

        // refresh the plot for every 5 incoming messages
        if (counter % 5 == 0)
        {
            cout << endl;
            cout.flush();

            printf("\033[%d;%dH", 1, 24);
            // system("clear");

            cout << table << endl;
            cout << "\n\n";

            cout << chart;
        }

        tmr.reset();
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_node;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Ros2PlotNode>());
    rclcpp::shutdown();
    return 0;
}
