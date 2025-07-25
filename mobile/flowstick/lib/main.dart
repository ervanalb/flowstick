import 'state.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: BlocProvider(
        create: (context) => MyCubit(),
        child: const MainPage(),
      ),
    );
  }
}

class MainPage extends StatelessWidget {
  const MainPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        centerTitle: true,
        title: Text("App"),
      ),
      body: BlocBuilder<MyCubit, MyState>(
        builder: (context, state) => switch (state) {
          Initializing() => const Center(
            child: CircularProgressIndicator(),
          ),
          Scanning(:final scanResults) => Column(children: [
            Text("Scanning"),
            Expanded(child: ListView.builder(
              itemCount: scanResults.length,
              itemBuilder: (context, index) {
                final result = scanResults[index];
                final resultName = result.device.platformName;
                return Card(
                  child: ListTile(
                    title: Text(resultName.isEmpty ? "Unknown Device" : resultName),
                    subtitle: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text("ID: ${result.device.remoteId}"),
                        Text("RSSI: ${result.rssi} dBm"),
                      ],
                    ),
                    trailing: ElevatedButton(
                      child: const Text("Connect"),
                      onPressed: () { print("CONNECT CLICKED"); },
                    ),
                  )
                );
              }
            )),
          ]),
          Connecting() => const Center(
            child: CircularProgressIndicator(),
          ),
          Connected(:final connectedState) => Center(child: Text("Connected!")),
          Error(:final errorMessage) => Center(child: Column(children: [
            SelectableText(errorMessage),
            ElevatedButton(child: const Text('Retry'), onPressed: () {context.read<MyCubit>().initialize();}),
          ])),
        },
      ),
    );
  }
}
