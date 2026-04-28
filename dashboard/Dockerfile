# Usamos Nginx Alpine por ser ligero y seguro
FROM nginx:alpine

# Copiamos los archivos de nuestra app al directorio que Nginx sirve por defecto
COPY . /usr/share/nginx/html

# Exponemos el puerto 80
EXPOSE 80

# Nginx se inicia automáticamente, no hace falta CMD
