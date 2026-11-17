#include <gtest/gtest.h>

#include <string>

#include "mowbite_setup/layer.h"

using mowbite_setup::Layer;
using json = Layer::json;

TEST(Layer, WithoutAValueThereIsNoFile) {
  Layer layer;
  EXPECT_EQ(layer.Text(), "");
  ASSERT_TRUE(layer.Set("xcore_driver", "inputs.learn", true));
  layer.Erase("xcore_driver", "inputs.learn");
  layer.Erase("nothing", "there");
  EXPECT_EQ(layer.Text(), "");
  EXPECT_TRUE(layer.Get("xcore_driver", "inputs.learn").is_null());
}

TEST(Layer, TheFileIsAParameterFile) {
  Layer layer;
  ASSERT_TRUE(layer.Set("xcore_driver", "inputs.gpio_lines", json::array({"GPIO1", "GPIO12"})));
  ASSERT_TRUE(layer.Set("xcore_driver", "inputs.gpio_active_high", json::array({true, false})));
  ASSERT_TRUE(layer.Set("xcore_driver", "inputs.gpio_delays_ms", json::array({10, 2500})));
  ASSERT_TRUE(layer.Set("xcore_driver", "inputs.learn", false));
  ASSERT_TRUE(layer.Set("mowbite_logic", "some.factor", 2.0));
  ASSERT_TRUE(layer.Set("mowbite_logic", "some.small", 1e-7));
  ASSERT_TRUE(layer.Set("mowbite_logic", "some.text", "a \"b\": # c"));
  const std::string text = layer.Text();
  const std::string values = text.substr(text.find("mowbite_logic:"));
  EXPECT_EQ(values,
            "mowbite_logic:\n"
            "  ros__parameters:\n"
            "    some.factor: 2.0\n"
            "    some.small: 1.0e-07\n"
            "    some.text: \"a \\\"b\\\": # c\"\n"
            "xcore_driver:\n"
            "  ros__parameters:\n"
            "    inputs.gpio_active_high: [true, false]\n"
            "    inputs.gpio_delays_ms: [10, 2500]\n"
            "    inputs.gpio_lines: [\"GPIO1\", \"GPIO12\"]\n"
            "    inputs.learn: false\n");
  EXPECT_EQ(text[0], '#');

  Layer read;
  std::string why;
  ASSERT_TRUE(read.Parse(text, &why)) << why;
  EXPECT_EQ(read.Text(), text);
  EXPECT_EQ(read.Get("xcore_driver", "inputs.gpio_delays_ms"), json::array({10, 2500}));
  EXPECT_EQ(read.Get("mowbite_logic", "some.text"), "a \"b\": # c");
  EXPECT_TRUE(read.Get("mowbite_logic", "some.factor").is_number_float());
}

TEST(Layer, OnlyWhatAParameterFileCanHold) {
  Layer layer;
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs.gpio_lines", json::array()));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs.gpio_lines", json::array({"GPIO1", 2})));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs.gpio_lines", json::array({json::array({1})})));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs", json::object()));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs", nullptr));
  EXPECT_FALSE(layer.Set("xcore_driver", "Inputs.x", 1));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs..x", 1));
  EXPECT_FALSE(layer.Set("xcore_driver", "inputs: 1\n    other", 1));
  EXPECT_FALSE(layer.Set("/**", "x", 1));
  EXPECT_FALSE(layer.Set("", "x", 1));
  EXPECT_EQ(layer.Text(), "");
}

TEST(Layer, AFileOfAnotherFormIsNotRead) {
  for (const char* text : {
           "xcore_driver:\n    inputs.learn: true\n",
           "  ros__parameters:\n    inputs.learn: true\n",
           "xcore_driver:\n  ros__parameters:\n    inputs:\n      learn: true\n",
           "xcore_driver:\n  ros__parameters:\n    inputs.learn: yes please\n",
           "xcore_driver:\n  ros__parameters:\n    inputs.gpio_lines: []\n",
           "xcore_driver: {ros__parameters: {inputs.learn: true}}\n",
           "/**:\n  ros__parameters:\n    inputs.learn: true\n",
       }) {
    Layer layer;
    ASSERT_TRUE(layer.Set("xcore_driver", "inputs.learn", true));
    std::string why;
    EXPECT_FALSE(layer.Parse(text, &why)) << text;
    EXPECT_NE(why.find("line "), std::string::npos) << text;
    EXPECT_EQ(layer.Text(), "") << text;
  }
  Layer empty;
  EXPECT_TRUE(empty.Parse("", nullptr));
  EXPECT_TRUE(empty.Parse("# only a comment\n\n", nullptr));
}
