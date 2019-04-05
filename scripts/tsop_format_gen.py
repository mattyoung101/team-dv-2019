print_str = "ESP_LOGD(TAG, \"Values: {}\", {})"
float_formats = ", ".join(["(%.2f, %.2f)"] * 18)
format_str = ", ".join([f"readings[{i}].X, readings[{i}].Y" for i in range(18)])

print(print_str.format(float_formats, format_str))