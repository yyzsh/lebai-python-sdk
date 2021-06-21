$name = "leibai-python-sdk-doc";
$dockerRegistryHost = "registry.cn-hangzhou.aliyuncs.com";
$dockerRegistryUrl = "${dockerRegistryHost}/ecy";
$dockerPassword = "xx";
$dockerPassword | docker login -u moeycy --password-stdin $dockerRegistryHost;


./make html;
docker build . -t ${name}:latest;

docker tag ${name}:latest "${dockerRegistryUrl}/${name}:latest";
docker push "${dockerRegistryUrl}/${name}:latest";

# python3 setup.py sdist bdist_wheel;

# python3 -m twine upload --repository pypi dist/*;
