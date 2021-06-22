
FROM sphinxdoc/sphinx AS sphinxdoc
ADD requirements.txt /docs
RUN pip3 install -r requirements.txt

FROM sphinxdoc AS build
WORKDIR /docs
COPY ./ .
RUN make html

FROM nginx AS final
EXPOSE 80 443
WORKDIR /
COPY --from=build /docs/build/html/  /usr/share/nginx/html/
COPY deploy/nginx.conf /etc/nginx/nginx.conf