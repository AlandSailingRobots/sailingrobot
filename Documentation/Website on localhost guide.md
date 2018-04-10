Website on localhost guide
==========================
The goal of this document is to describe the installation of all needed services to run the database on an Arch Linux workstation.

## Installation and first time setup
Install and enable apache, mysql (mariadb), PHP and phpMyAdmin

1. Install
    ```console
    # pacman -S apache mariadb php php-fpm phpmyadmin
    ```

2. Set up mariadb
    ```console
    # mysql_install_db --user=mysql --basedir=/usr --datadir=/var/lib/mysql
    ```

* **Note:** The default database credentials are root without password

3. Edit */etc/php/php.ini*:
    ```sh
    ...
    date.timezone = Europe/Mariehamn
    display_errors = On
    extension=bz2
    extension=mysqli
    extension=pdo_mysql
    ...
    ```

4. Create */etc/httpd/conf/extra/php-fpm.conf*
    ```html
    DirectoryIndex index.php index.html
    <FilesMatch \.php$>
        SetHandler "proxy:unix:/run/php-fpm/php-fpm.sock|fcgi://localhost/"
    </FilesMatch>
    ```

5. Create */etc/httpd/conf/extra/phpmyadmin.conf*
    ```html
    Alias /phpmyadmin "/usr/share/webapps/phpMyAdmin"
    <Directory "/usr/share/webapps/phpMyAdmin">
        DirectoryIndex index.php
        AllowOverride All
        Options FollowSymlinks
        Require all granted
    </Directory>
    ```

6. Permit empty passwords in phpMyAdmin by editing */etc/webapps/phpmyadmin/config.inc.php*
    ```php
    $cfg['Servers'][$i]['AllowNoPassword'] = true;
    ```

7. Include the previous configs in */etc/httpd/conf/httpd.conf*
    ```sh
    ...
    # Load modules for proxy
    LoadModule proxy_module modules/mod_proxy.so
    LoadModule proxy_fcgi_module modules/mod_proxy_fcgi.so
    
    # php-fpm configuration
    Include conf/extra/php-fpm.conf
    
    # phpMyAdmin configuration
    Include conf/extra/phpmyadmin.conf
    ...
    ```

## Starting all services on local machine
Start everything up (it might be overkill to enable all services at startup but the downside is that this step needs to be done every time you need the services locally)

  ```console
  # systemctl start httpd mariadb php-fpm
  ```

## Magically obtain the project database ...

## References:
  * https://wiki.archlinux.org/index.php/Apache_HTTP_Server
  * https://wiki.archlinux.org/index.php/MySQL
  * https://wiki.archlinux.org/index.php/PHP
  * https://wiki.archlinux.org/index.php/phpMyAdmin
  * Download DB from HostGator

