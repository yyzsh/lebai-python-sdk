FROM nginx
EXPOSE 80 443
WORKDIR /
COPY build/html/  /usr/share/nginx/html/
COPY deploy/nginx.conf /etc/nginx/nginx.conf