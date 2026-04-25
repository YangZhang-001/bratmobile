#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();

	series = new QScatterSeries();
	series->setMarkerSize(10.0);

	QVector<QPointF> lidarPoints = {
		{1.0, 2.0},
		{2.0, 3},
		{3, 1}};

	for (const QPointF &p : lidarPoints)
		series->append(p);

	chart = new QChart();
	chart->addSeries(series);
	chart->createDefaultAxes();
	auto xAxis = chart->axes(Qt::Horizontal);
	auto yAxis = chart->axes(Qt::Vertical);
	xAxis.back()->setRange(-5, 5);
	yAxis.back()->setRange(-5, 5);
	chart->setTitle("LiDAR XY Plot");

	chartView = new QChartView(chart);

	vLayout->addWidget(chartView);

	setLayout(vLayout);

	fprintf(stderr, "Starting screen update timer.\n");
	startTimer(std::chrono::milliseconds{100});
}

void Window::timerEvent(QTimerEvent *)
{
	QList<QPointF> points;
	const double w = sin(t);
	const double h = cos(t);
	for (int i = 0; i < 100; i++)
	{
		const double x = ((double)rand() * 10 / RAND_MAX - 5)*w;
		const double y = ((double)rand() * 10 / RAND_MAX - 5)*h;
		points.append({x,y});
	}
	series->replace(points);
	t = t + 0.1;
}
