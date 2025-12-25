#!/bin/bash

ESP_IP="192.168.1.122"
DEVICE="$2"
CHARACTERISTIC="$3"
VALUE="$4"

# Hàm lấy trạng thái từ ESP
get_status() {
    curl -s "http://${ESP_IP}/status"
}

# Hàm gửi lệnh điều khiển
send_control() {
    local data="$1"
    curl -s -X POST "http://${ESP_IP}/control" \
         -H "Content-Type: application/json" \
         -d "$data"
}

case "$1" in
    Get)
        STATUS=$(get_status)
        
        case "$DEVICE" in
            "Máy Lạnh")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    POWER=$(echo "$STATUS" | grep -o '"power":"[^"]*"' | cut -d'"' -f4)
                    [ "$POWER" = "on" ] && echo "1" || echo "0"
                fi
                ;;
                
            "Nhiệt Độ Máy Lạnh")
                if [ "$CHARACTERISTIC" = "CurrentTemperature" ]; then
                    echo "$STATUS" | grep -o '"temp":[0-9]*' | cut -d':' -f2
                fi
                ;;
                
            "Điều Chỉnh Máy Lạnh")
                case "$CHARACTERISTIC" in
                    "CurrentHeatingCoolingState")
                        POWER=$(echo "$STATUS" | grep -o '"power":"[^"]*"' | cut -d'"' -f4)
                        [ "$POWER" = "on" ] && echo "2" || echo "0"
                        ;;
                    "TargetHeatingCoolingState")
                        echo "2"  # Always COOL mode
                        ;;
                    "CurrentTemperature")
                        echo "$STATUS" | grep -o '"temp":[0-9]*' | cut -d':' -f2
                        ;;
                    "TargetTemperature")
                        echo "$STATUS" | grep -o '"temp":[0-9]*' | cut -d':' -f2
                        ;;
                    "TemperatureDisplayUnits")
                        echo "0"  # Celsius
                        ;;
                esac
                ;;
                
            "Máy Lạnh Swing Dọc")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    SWING=$(echo "$STATUS" | grep -o '"swing_v":[a-z]*' | cut -d':' -f2)
                    [ "$SWING" = "true" ] && echo "1" || echo "0"
                fi
                ;;
                
            "Học Lệnh IR")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    echo "0"  # Always return OFF for stateless switch
                fi
                ;;
        esac
        ;;
        
    Set)
        case "$DEVICE" in
            "Máy Lạnh")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    [ "$VALUE" = "1" ] && POWER="on" || POWER="off"
                    send_control "{\"power\":\"$POWER\"}"
                    echo "0"  # Success
                fi
                ;;
                
            "Điều Chỉnh Máy Lạnh")
                case "$CHARACTERISTIC" in
                    "TargetHeatingCoolingState")
                        # 0=OFF, 1=HEAT, 2=COOL, 3=AUTO
                        if [ "$VALUE" = "0" ]; then
                            send_control "{\"power\":\"off\"}"
                        else
                            send_control "{\"power\":\"on\"}"
                        fi
                        echo "0"
                        ;;
                    "TargetTemperature")
                        TEMP=$(printf "%.0f" "$VALUE")
                        send_control "{\"temp\":$TEMP}"
                        echo "0"
                        ;;
                esac
                ;;
                
            "Máy Lạnh Swing Dọc")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    [ "$VALUE" = "1" ] && SWING="true" || SWING="false"
                    send_control "{\"swing_v\":$SWING}"
                    echo "0"
                fi
                ;;
                
            "Học Lệnh IR")
                if [ "$CHARACTERISTIC" = "On" ]; then
                    curl -s "http://${ESP_IP}/learn" > /dev/null
                    echo "0"
                fi
                ;;
        esac
        ;;
esac

exit 0