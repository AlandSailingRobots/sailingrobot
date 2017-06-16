    
    #      Aland Sailing Robot       #
    # RaspberryPi Management Scripts #
    #--------------------------------#
    # Deamon install/remove
    
    source repos.sh
    
    opt_daemon_create ()
    {
            echo "[Unit]" >> asr.service
            echo "Description=sailingrobot" >> asr.service
            echo "Requires=gpsd.service" >> asr.service
            echo "After=gpsd.service" >> asr.service
	    echo "After=network.target" >> asr.service
            echo "" >> asr.service
            echo "[Service]" >> asr.service
            echo "Type=simple" >> asr.service
            echo "ExecStart=$INSTALLATION_PATH/$SAILBOT_REPO_MAIN/sr" >> asr.service
            echo "RestartSec=5" >> asr.service
            echo "Restart=on-failure" >> asr.service
            echo "KillSignal=SIGINT" >> asr.service
            echo "" >> asr.service
            echo "[Install]" >> asr.service
            echo "WantedBy=multi-user.target" >> asr.service
            
            printf "$CLR_INFO\nDaemon created\n"
            sudo mv asr.service /etc/systemd/system/asr.service
            sudo systemctl enable asr.service
            rm -rf asr.service
            printf "$CLR_INFO\nDaemon enabled\n"
    }

    opt_daemon_remove ()
    {
            sudo systemctl disable asr.service
            printf "$CLR_INFO\nDaemon disabled\n"
            sudo rm /etc/systemd/system/asr.service
            printf "$CLR_INFO"
            printf "Daemon removed\n"
    }

    printf "\n$CLR_ASK\nWhat do you want to do?\n$CLR_OPT"
    select option in "Create Daemon" "Remove Daemon" "Back"
    do
            case $option in
                    "Create Daemon" ) opt_daemon_create; break;;
                    "Remove Daemon" ) opt_daemon_remove; break;;
                    "Back" ) break;;
            esac
    done

    # Return to menu
    cd $PI_SHELL_PATH
    ./menu.sh
